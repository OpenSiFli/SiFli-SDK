/**
  ******************************************************************************
  * @file   bf0_hal_def.h
  * @author Sifli software development team
  * @brief   This file contains HAL common defines, enumeration, macros and
  *          structures definitions.
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

#ifndef __BF0_HAL_DEF
#define __BF0_HAL_DEF

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup COMMON Common
  * @brief Common HAL common type definition
  * @{
  */


/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <stdbool.h>
/* Exported types ------------------------------------------------------------*/

/**
  * @brief  HAL Status structures definition
  */
typedef enum
{
    HAL_OK       = 0x00, /**< No error */
    HAL_ERROR    = 0x01, /**< General error */
    HAL_BUSY     = 0x02, /**< Busy */
    HAL_TIMEOUT  = 0x03,  /**< Timeout */

    HAL_EPIC_NOTHING_TO_DO = 0x10
} HAL_StatusTypeDef;

/**
  * @brief  HAL Lock structures definition
  */
typedef enum
{
    HAL_UNLOCKED = 0x00,
    HAL_LOCKED   = 0x01
} HAL_LockTypeDef;

#ifndef WIN32
#include "register.h"
#endif

/* Exported macros -----------------------------------------------------------*/

#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */

#define HAL_MAX_DELAY      0xFFFFFFFFU

#define HAL_IS_BIT_SET(REG, BIT)         (((REG) & (BIT)) == (BIT))
#define HAL_IS_BIT_CLR(REG, BIT)         (((REG) & (BIT)) == 0U)

#define __HAL_LINKDMA(__HANDLE__, __PPP_DMA_FIELD__, __DMA_HANDLE__)             \
                        do{                                                      \
                            (__HANDLE__)->__PPP_DMA_FIELD__ = &(__DMA_HANDLE__); \
                            (__DMA_HANDLE__).Parent = (__HANDLE__);              \
                        } while(0)

/** @brief Reset the Handle's State field.
  * @param  \__HANDLE__: specifies the Peripheral Handle.
  * @note  This macro can be used for the following purpose:
  *          - When the Handle is declared as local variable; before passing it as parameter
  *            to HAL_PPP_Init() for the first time, it is mandatory to use this macro
  *            to set to 0 the Handle's "State" field.
  *            Otherwise, "State" field may have any random value and the first time the function
  *            HAL_PPP_Init() is called, the low level hardware initialization will be missed
  *            (i.e. HAL_PPP_MspInit() will not be executed).
  *          - When there is a need to reconfigure the low level hardware: instead of calling
  *            HAL_PPP_DeInit() then HAL_PPP_Init(), user can make a call to this macro then HAL_PPP_Init().
  *            In this later function, when the Handle's "State" field is set to 0, it will execute the function
  *            HAL_PPP_MspInit() which will reconfigure the low level hardware.
  * @retval None
  */
#define __HAL_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = 0)

#define __HAL_LOCK(__HANDLE__)                                           \
                                do{                                        \
                                    if((__HANDLE__)->Lock == HAL_LOCKED)   \
                                    {                                      \
                                       return HAL_BUSY;                    \
                                    }                                      \
                                    else                                   \
                                    {                                      \
                                       (__HANDLE__)->Lock = HAL_LOCKED;    \
                                    }                                      \
                                  }while (0)

#define __HAL_UNLOCK(__HANDLE__)                                          \
                                  do{                                       \
                                      (__HANDLE__)->Lock = HAL_UNLOCKED;    \
                                    }while (0)

#if defined ( __GNUC__ ) && !defined (__CC_ARM) /* GNU Compiler */
#ifndef __weak
#define __weak   __attribute__((weak))
#endif /* __weak */
#ifndef __packed
#define __packed __attribute__((__packed__))
#endif /* __packed */
#endif /* __GNUC__ */


/* Macro to get variable aligned on 4-bytes, for __ICCARM__ the directive "#pragma data_alignment=4" must be used instead */
#if defined ( __GNUC__ ) && !defined (__CC_ARM) /* GNU Compiler */
#ifndef __ALIGN_END
#define __ALIGN_END    __attribute__ ((aligned (4)))
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#define __ALIGN_BEGIN
#endif /* __ALIGN_BEGIN */
#else
#ifndef __ALIGN_END
#define __ALIGN_END
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#if defined   (__CC_ARM)      /* ARM Compiler */
#define __ALIGN_BEGIN    __align(4)
#elif defined (__ICCARM__)    /* IAR Compiler */
#define __ALIGN_BEGIN
#endif /* __CC_ARM */
#endif /* __ALIGN_BEGIN */
#endif /* __GNUC__ */

/**
  * @brief  __RAM_FUNC definition
  */
#if defined ( __CC_ARM   )
/* ARM Compiler
   ------------
   RAM functions are defined using the toolchain options.
   Functions that are executed in RAM should reside in a separate source module.
   Using the 'Options for File' dialog you can simply change the 'Code / Const'
   area of a module to a memory space in physical RAM.
   Available memory areas are declared in the 'Target' tab of the 'Options for Target'
   dialog.
*/
#define __RAM_FUNC HAL_StatusTypeDef

#elif defined ( __ICCARM__ )
/* ICCARM Compiler
   ---------------
   RAM functions are defined using a specific toolchain keyword "__ramfunc".
*/
#define __RAM_FUNC __ramfunc HAL_StatusTypeDef

#elif defined   (  __GNUC__  )
/* GNU Compiler
   ------------
  RAM functions are defined using a specific toolchain attribute
   "__attribute__((section(".RamFunc")))".
*/
#define __RAM_FUNC HAL_StatusTypeDef  __attribute__((section(".RamFunc")))

#endif

#if !defined(__CLANG_ARM) && defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#define __CLANG_ARM
#endif

#ifndef HAL_SECTION
#if defined(__CC_ARM) || defined(__CLANG_ARM)           /* ARM Compiler */
#define HAL_SECTION(x)                  __attribute__((section(x)))
#elif defined (__IAR_SYSTEMS_ICC__)     /* for IAR Compiler */
#define HAL_SECTION(x)                  @ x
#elif defined (__GNUC__)                /* GNU GCC Compiler */
#define HAL_SECTION(x)                  __attribute__((section(x)))
#else
#define HAL_SECTION(x)
#endif
#endif

#define HAL_STRINGIFY_(val)       #val

/** Converts a macro argument into a character constant.
 */
#define HAL_STRINGIFY(val)       HAL_STRINGIFY_(val)

/** RAM non-retained code section */
#if  defined(_MSC_VER)
#define HAL_RAM_NON_RET_CODE_SECT(section_name, func)        func
#elif defined(__IAR_SYSTEMS_ICC__)
#define HAL_RAM_NON_RET_CODE_SECT(section_name, func)        func HAL_SECTION(HAL_STRINGIFY(.l1_non_ret_text_##section_name))
#else
#define HAL_RAM_NON_RET_CODE_SECT(section_name, func)        HAL_SECTION(HAL_STRINGIFY(.l1_non_ret_text_##section_name)) func
#endif /* _MSC_VER */


/** RAM non-retained rodata section */
#define HAL_RAM_NON_RET_RODATA_SECT(section_name)        HAL_SECTION(HAL_STRINGIFY(.l1_non_ret_rodata_##section_name))

/** RAM retained code section */
#if  defined(_MSC_VER)
#define HAL_RAM_RET_CODE_SECT(section_name, func)        func
#elif defined(__IAR_SYSTEMS_ICC__)
#define HAL_RAM_RET_CODE_SECT(section_name, func)        func HAL_SECTION(HAL_STRINGIFY(.l1_ret_text_##section_name))
#else
#define HAL_RAM_RET_CODE_SECT(section_name, func)        HAL_SECTION(HAL_STRINGIFY(.l1_ret_text_##section_name)) func
#endif /* _MSC_VER */

/** RAM retained rodata section */
#define HAL_RAM_RET_RODATA_SECT(section_name)            HAL_SECTION(HAL_STRINGIFY(.l1_ret_rodata_##section_name))

#if defined(__CC_ARM) || defined(__CLANG_ARM)
#define HAL_RETM_BSS_SECT(section_name, var)             var HAL_SECTION(HAL_STRINGIFY(.bss.retm_bss_##section_name))
#elif  defined(_MSC_VER)
#define HAL_RETM_BSS_SECT(section_name, var)             var
#else
#define HAL_RETM_BSS_SECT(section_name, var)             var HAL_SECTION(HAL_STRINGIFY(.bss.retm_bss_##section_name))
#endif /* __CC_ARM  */


/**
  * @brief  __NOINLINE definition
  */
#if defined ( __CC_ARM   ) || defined   (  __GNUC__  )
/* ARM & GNUCompiler
   ----------------
*/
#define __NOINLINE __attribute__ ( (noinline) )

#elif defined ( __ICCARM__ )
/* ICCARM Compiler
   ---------------
*/
#define __NOINLINE _Pragma("optimize = no_inline")

#endif


/**
  * @}
  */

/**
  * @}
  */



#ifdef __cplusplus
}
#endif

#endif /* ___BF0_HAL_DEF */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

