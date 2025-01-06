/**
  ******************************************************************************
  * @file   pwm_config.h
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

#ifndef __PWM_CONFIG_H__
#define __PWM_CONFIG_H__

#include <rtconfig.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PWM2_CORE CORE_ID_HCPU
#define PWM3_CORE CORE_ID_HCPU
#define PWM4_CORE CORE_ID_LCPU
#define PWM5_CORE CORE_ID_LCPU
#define PWM6_CORE CORE_ID_LCPU




#ifdef BSP_USING_PWM2
#define PWM2_CONFIG                             \
    {                                           \
       .tim_handle.Instance     = GPTIM1,       \
       .tim_handle.core         = PWM2_CORE,    \
       .name                    = "pwm2",       \
       .channel                 = 0             \
    }
#endif /* BSP_USING_PWM2 */

#ifdef BSP_USING_PWM3
#define PWM3_CONFIG                             \
    {                                           \
       .tim_handle.Instance     = GPTIM2,         \
       .tim_handle.core         = PWM3_CORE,    \
       .name                    = "pwm3",       \
       .channel                 = 0             \
    }
#endif /* BSP_USING_PWM3 */

#if defined(BSP_USING_RGBLED_WITCH_PWM3)
#define RGBLED_CONFIG                             \
        {                                           \
           .tim_handle.Instance     = GPTIM2,         \
           .tim_handle.core         = PWM3_CORE,    \
           .name                    = "rgbled",       \
           .channel                 = 1,             \
           .dma_handle.Init.Priority = PWM3_CC1_DMA_IRQ_PRIO,           \
           .dma_handle.Instance = PWM3_CC1_DMA_INSTANCE,               \
           .dma_irq = PWM3_CC1_DMA_IRQ,                     \
           .dma_handle.Init.Request = PWM3_CC1_DMA_REQUEST                 \
        }
#endif /* BSP_USING_PWM3 */



#ifdef BSP_USING_PWM4
#define PWM4_CONFIG                             \
    {                                           \
       .tim_handle.Instance     = GPTIM3,         \
       .tim_handle.core         = PWM4_CORE,    \
       .name                    = "pwm4",       \
       .channel                 = 0             \
    }
#endif /* BSP_USING_PWM4 */

#ifdef BSP_USING_PWM5
#define PWM5_CONFIG                             \
    {                                           \
       .tim_handle.Instance     = GPTIM4,         \
       .tim_handle.core         = PWM5_CORE,    \
       .name                    = "pwm5",       \
       .channel                 = 0             \
    }
#endif /* BSP_USING_PWM5 */

#ifdef BSP_USING_PWM6
#define PWM6_CONFIG                             \
    {                                           \
       .tim_handle.Instance     = GPTIM5,         \
       .tim_handle.core         = PWM6_CORE,    \
       .name                    = "pwm6",       \
       .channel                 = 0             \
    }
#endif /* BSP_USING_PWM5 */

#ifdef BSP_USING_PWMA1
#define PWMA1_CONFIG                             \
        {                                           \
           .tim_handle.Instance     = (GPT_TypeDef *)ATIM1,         \
           .tim_handle.core         = ATIM1_CORE,    \
           .name                    = "pwma1",       \
           .channel                 = 0             \
        }
#endif /* BSP_USING_PWM_A1 */

#ifdef BSP_USING_PWMA2
#define PWMA2_CONFIG                             \
        {                                           \
           .tim_handle.Instance     = (GPT_TypeDef *)ATIM2,         \
           .tim_handle.core         = ATIM2_CORE,    \
           .name                    = "pwma2",       \
           .channel                 = 0             \
        }
#endif /* BSP_USING_PWM_A2 */

#ifdef BSP_USING_PWM_LPTIM1
#define PWM_LPTIM1_CONFIG                       \
    {                                           \
       .tim_handle.Instance     = hwp_lptim1,   \
       .name                    = "pwmlp1",       \
    }

#endif /* BSP_USING_PWM_LPTIM1 */

#ifdef BSP_USING_PWM_LPTIM2
#define PWM_LPTIM2_CONFIG                       \
    {                                           \
       .tim_handle.Instance     = hwp_lptim2,   \
       .name                    = "pwmlp2",       \
    }
#endif /* BSP_USING_PWM_LPTIM2 */

#ifdef BSP_USING_PWM_LPTIM3
#define PWM_LPTIM3_CONFIG                       \
    {                                           \
       .tim_handle.Instance     = hwp_lptim3,   \
       .name                    = "pwmlp3",       \
    }
#endif /* BSP_USING_PWM_LPTIM3 */


#ifdef __cplusplus
}
#endif

#endif /* __PWM_CONFIG_H__ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
