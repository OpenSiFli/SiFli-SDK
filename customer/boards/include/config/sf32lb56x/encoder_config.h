/**
  ******************************************************************************
  * @file   encoder_config.h
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

#ifndef __ENCODER_CONFIG_H__
#define __ENCODER_CONFIG_H__

#include <rtconfig.h>

#ifdef __cplusplus
extern "C" {
#endif


#ifdef BSP_USING_ENCODER_GPTIM1
#define ENCODER_GPTIM1_CONFIG                             \
    {                                               \
        .tim_handle.Instance     = GPTIM1,          \
        .tim_handle.core         = CORE_ID_HCPU,   \
        .name                    = "encoder1",      \
        .encoder_init.EncoderMode         = GPT_ENCODERMODE_TI3, \
        .encoder_init.IC1Polarity         = GPT_ICPOLARITY_RISING, \
        .encoder_init.IC1Selection        = GPT_ICSELECTION_DIRECTTI, \
        .encoder_init.IC1Prescaler        = GPT_ICPSC_DIV1,  \
        .encoder_init.IC1Filter           = 0xF,               \
        .encoder_init.IC2Polarity         = GPT_ICPOLARITY_RISING, \
        .encoder_init.IC2Selection        = GPT_ICSELECTION_DIRECTTI, \
        .encoder_init.IC2Prescaler        = GPT_ICPSC_DIV1,  \
        .encoder_init.IC2Filter           = 0xF                \
                                               \
    }
#endif /* BSP_USING_ENCODER_GPTIM1 */


#ifdef BSP_USING_ENCODER_GPTIM2
#define ENCODER_GPTIM2_CONFIG                             \
    {                                               \
        .tim_handle.Instance     = GPTIM2,          \
        .tim_handle.core         = CORE_ID_HCPU,   \
        .name                    = "encoder2",      \
        .encoder_init.EncoderMode         = GPT_ENCODERMODE_TI3, \
        .encoder_init.IC1Polarity         = GPT_ICPOLARITY_RISING, \
        .encoder_init.IC1Selection        = GPT_ICSELECTION_DIRECTTI, \
        .encoder_init.IC1Prescaler        = GPT_ICPSC_DIV1,  \
        .encoder_init.IC1Filter           = 0xF,               \
        .encoder_init.IC2Polarity         = GPT_ICPOLARITY_RISING, \
        .encoder_init.IC2Selection        = GPT_ICSELECTION_DIRECTTI, \
        .encoder_init.IC2Prescaler        = GPT_ICPSC_DIV1,  \
        .encoder_init.IC2Filter           = 0xF                \
                                               \
    }
#endif /* BSP_USING_ENCODER_GPTIM2 */


#ifdef BSP_USING_ENCODER_GPTIM3
#define ENCODER_GPTIM3_CONFIG                             \
    {                                               \
        .tim_handle.Instance     = GPTIM3,          \
        .tim_handle.core         = CORE_ID_LCPU,   \
        .name                    = "encoder3",      \
        .encoder_init.EncoderMode         = GPT_ENCODERMODE_TI3, \
        .encoder_init.IC1Polarity         = GPT_ICPOLARITY_RISING, \
        .encoder_init.IC1Selection        = GPT_ICSELECTION_DIRECTTI, \
        .encoder_init.IC1Prescaler        = GPT_ICPSC_DIV1,  \
        .encoder_init.IC1Filter           = 0xF,               \
        .encoder_init.IC2Polarity         = GPT_ICPOLARITY_RISING, \
        .encoder_init.IC2Selection        = GPT_ICSELECTION_DIRECTTI, \
        .encoder_init.IC2Prescaler        = GPT_ICPSC_DIV1,  \
        .encoder_init.IC2Filter           = 0xF                \
                                               \
    }
#endif /* BSP_USING_ENCODER_GPTIM3 */

#ifdef BSP_USING_ENCODER_GPTIM4
#define ENCODER_GPTIM4_CONFIG                             \
    {                                               \
        .tim_handle.Instance     = GPTIM4,          \
        .tim_handle.core         = CORE_ID_LCPU,   \
        .name                    = "encoder4",      \
        .encoder_init.EncoderMode         = GPT_ENCODERMODE_TI3, \
        .encoder_init.IC1Polarity         = GPT_ICPOLARITY_RISING, \
        .encoder_init.IC1Selection        = GPT_ICSELECTION_DIRECTTI, \
        .encoder_init.IC1Prescaler        = GPT_ICPSC_DIV1,  \
        .encoder_init.IC1Filter           = 0xF,               \
        .encoder_init.IC2Polarity         = GPT_ICPOLARITY_RISING, \
        .encoder_init.IC2Selection        = GPT_ICSELECTION_DIRECTTI, \
        .encoder_init.IC2Prescaler        = GPT_ICPSC_DIV1,  \
        .encoder_init.IC2Filter           = 0xF                \
                                               \
    }
#endif /* BSP_USING_ENCODER_GPTIM4 */

#ifdef BSP_USING_ENCODER_GPTIM5
#define ENCODER_GPTIM5_CONFIG                             \
    {                                               \
        .tim_handle.Instance     = GPTIM5,          \
        .tim_handle.core         = CORE_ID_LCPU,   \
        .name                    = "encoder5",      \
        .encoder_init.EncoderMode         = GPT_ENCODERMODE_TI3, \
        .encoder_init.IC1Polarity         = GPT_ICPOLARITY_RISING, \
        .encoder_init.IC1Selection        = GPT_ICSELECTION_DIRECTTI, \
        .encoder_init.IC1Prescaler        = GPT_ICPSC_DIV1,  \
        .encoder_init.IC1Filter           = 0xF,               \
        .encoder_init.IC2Polarity         = GPT_ICPOLARITY_RISING, \
        .encoder_init.IC2Selection        = GPT_ICSELECTION_DIRECTTI, \
        .encoder_init.IC2Prescaler        = GPT_ICPSC_DIV1,  \
        .encoder_init.IC2Filter           = 0xF                \
                                               \
    }
#endif /* BSP_USING_ENCODER_GPTIM5 */

#ifdef BSP_USING_ENCODER_ATIM1
#define ENCODER_ATIM1_CONFIG                             \
    {                                               \
        .tim_handle.Instance     = ATIM1,          \
        .tim_handle.core         = CORE_ID_HCPU,   \
        .name                    = "Aencoder1",      \
        .encoder_init.EncoderMode         = GPT_ENCODERMODE_TI3, \
        .encoder_init.IC1Polarity         = GPT_ICPOLARITY_RISING, \
        .encoder_init.IC1Selection        = GPT_ICSELECTION_DIRECTTI, \
        .encoder_init.IC1Prescaler        = GPT_ICPSC_DIV1,  \
        .encoder_init.IC1Filter           = 0xF,               \
        .encoder_init.IC2Polarity         = GPT_ICPOLARITY_RISING, \
        .encoder_init.IC2Selection        = GPT_ICSELECTION_DIRECTTI, \
        .encoder_init.IC2Prescaler        = GPT_ICPSC_DIV1,  \
        .encoder_init.IC2Filter           = 0xF                \
                                               \
    }
#endif /* BSP_USING_ENCODER_ATIM1 */






#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_CONFIG_H__ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
