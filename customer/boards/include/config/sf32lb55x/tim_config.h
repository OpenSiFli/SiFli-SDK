/**
  ******************************************************************************
  * @file   tim_config.h
  * @author Sifli software development team
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

#ifndef __TIM_CONFIG_H__
#define __TIM_CONFIG_H__

#include <rtconfig.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef LXT_FREQ
#define LXT_FREQ 32768
#endif


#define GPTIM1_CORE CORE_ID_HCPU
#define GPTIM2_CORE CORE_ID_HCPU
#define GPTIM3_CORE CORE_ID_LCPU
#define GPTIM4_CORE CORE_ID_LCPU
#define GPTIM5_CORE CORE_ID_LCPU
#define ATIM1_CORE CORE_ID_HCPU
#define ATIM2_CORE CORE_ID_HCPU
#define BTIM1_CORE CORE_ID_HCPU
#define BTIM2_CORE CORE_ID_HCPU
#define BTIM3_CORE CORE_ID_LCPU
#define BTIM4_CORE CORE_ID_LCPU
#define LPTIM1_CORE CORE_ID_HCPU
#define LPTIM2_CORE CORE_ID_LCPU
#define LPTIM3_CORE CORE_ID_LCPU




#ifndef GPT_DEV_INFO_CONFIG
#define GPT_DEV_INFO_CONFIG                     \
    {                                           \
        .maxfreq = 1000000,                     \
        .minfreq = 2000,                        \
        .maxcnt  = 0xFFFF,                      \
        .cntmode = HWTIMER_CNTMODE_UP,          \
    }
#endif /* TIM_DEV_INFO_CONFIG */

#ifndef LPTIM_DEV_INFO_CONFIG
#define LPTIM_MAX_CNT 0xFFFFFF
#define LPTIM_DEV_INFO_CONFIG                   \
    {                                           \
        .maxfreq = LXT_FREQ,                    \
        .minfreq = 256,                         \
        .maxcnt  = LPTIM_MAX_CNT,               \
        .cntmode = HWTIMER_CNTMODE_UP,          \
    }
#endif /* TIM_DEV_INFO_CONFIG */


#ifdef BSP_USING_GPTIM1
#define GPTIM1_CONFIG                           \
    {                                           \
       .tim_handle.Instance     = GPTIM1,       \
       .tim_irqn                = GPTIM1_IRQn,  \
       .name                    = "gptim1",     \
       .core                    = GPTIM1_CORE, \
    }
#endif /* BSP_USING_GPTIM1 */

#ifdef BSP_USING_GPTIM2
#define GPTIM2_CONFIG                           \
    {                                           \
       .tim_handle.Instance     = GPTIM2,       \
       .tim_irqn                = GPTIM2_IRQn,  \
       .name                    = "gptim2",     \
       .core                    = GPTIM2_CORE, \
    }
#endif /* BSP_USING_GPTIM2 */

#ifdef BSP_USING_GPTIM3
#define GPTIM3_CONFIG                           \
    {                                           \
       .tim_handle.Instance     = GPTIM3,       \
       .tim_irqn                = GPTIM3_IRQn,  \
       .name                    = "gptim3",     \
       .core                    = GPTIM3_CORE, \
    }
#endif /* BSP_USING_GPTIM3 */

#ifdef BSP_USING_GPTIM4
#define GPTIM4_CONFIG                           \
    {                                           \
       .tim_handle.Instance     = GPTIM4,       \
       .tim_irqn                = GPTIM4_IRQn,  \
       .name                    = "gptim4",     \
       .core                    = GPTIM4_CORE, \
    }
#endif /* BSP_USING_GPTIM4 */

#ifdef BSP_USING_GPTIM5
#define GPTIM5_CONFIG                           \
    {                                           \
       .tim_handle.Instance     = GPTIM5,       \
       .tim_irqn                = GPTIM5_IRQn,  \
       .name                    = "gptim5",     \
       .core                    = GPTIM5_CORE, \
    }
#endif /* BSP_USING_GPTIM5 */

#ifdef BSP_USING_ATIM1
#define ATIM1_CONFIG                            \
        {                                           \
           .tim_handle.Instance     = (GPT_TypeDef *)ATIM1,         \
           .tim_irqn                = ATIM1_IRQn,   \
           .name                    = "atim1",      \
           .core                    = ATIM1_CORE,   \
        }
#endif /* BSP_USING_BTIM1 */

#ifdef BSP_USING_ATIM2
#define ATIM2_CONFIG                             \
        {                                            \
           .tim_handle.Instance     = (GPT_TypeDef *)ATIM2,         \
           .tim_irqn                = ATIM2_IRQn,    \
           .name                    = "atim2",       \
           .core                    = ATIM2_CORE,   \
        }
#endif /* BSP_USING_BTIM2 */

#ifdef BSP_USING_BTIM1
#define BTIM1_CONFIG                            \
    {                                           \
       .tim_handle.Instance     = (GPT_TypeDef *)BTIM1,         \
       .tim_irqn                = BTIM1_IRQn,   \
       .name                    = "btim1",      \
       .core                    = BTIM1_CORE,   \
    }
#endif /* BSP_USING_BTIM1 */

#ifdef BSP_USING_BTIM2
#define BTIM2_CONFIG                             \
    {                                            \
       .tim_handle.Instance     = (GPT_TypeDef *)BTIM2,         \
       .tim_irqn                = BTIM2_IRQn,    \
       .name                    = "btim2",       \
       .core                    = BTIM2_CORE,   \
    }
#endif /* BSP_USING_BTIM2 */

#ifdef BSP_USING_BTIM3
#define BTIM3_CONFIG                             \
    {                                            \
       .tim_handle.Instance     = (GPT_TypeDef *)BTIM3,         \
       .tim_irqn                = BTIM3_IRQn,    \
       .name                    = "btim3",       \
       .core                    = BTIM3_CORE,    \
    }
#endif /* BSP_USING_BTIM3 */

#ifdef BSP_USING_BTIM4
#define BTIM4_CONFIG                            \
    {                                           \
       .tim_handle.Instance     = (GPT_TypeDef *)BTIM4,         \
       .tim_irqn                = BTIM4_IRQn,   \
       .name                    = "btim4",      \
       .core                    = BTIM4_CORE,    \
    }
#endif /* BSP_USING_BTIM4 */


#ifdef BSP_USING_LPTIM1
#define LPTIM1_CONFIG                             \
    {                                            \
       .tim_handle.Instance     = hwp_lptim1,        \
       .tim_irqn                = LPTIM1_IRQn,   \
       .name                    = "lptim1",     \
    }
#endif /* BSP_USING_LPTIM1 */

#ifdef BSP_USING_LPTIM2
#define LPTIM2_CONFIG                             \
    {                                            \
       .tim_handle.Instance     = hwp_lptim2,    \
       .tim_irqn                = LPTIM2_IRQn,   \
       .name                    = "lptim2",     \
    }
#endif /* BSP_USING_LPTIM2 */

#ifdef BSP_USING_LPTIM3
#define LPTIM3_CONFIG                             \
    {                                            \
       .tim_handle.Instance     = hwp_lptim3,        \
       .tim_irqn                = LPTIM3_IRQn,   \
       .name                    = "lptim3",     \
    }
#endif /* BSP_USING_LPTIM3 */


#ifdef __cplusplus
}
#endif

#endif /* __TIM_CONFIG_H__ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
