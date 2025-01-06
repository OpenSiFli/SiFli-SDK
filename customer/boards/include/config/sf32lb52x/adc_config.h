/**
  ******************************************************************************
  * @file   adc_config.h
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

#ifndef __ADC_CONFIG_H__
#define __ADC_CONFIG_H__

#include <rtconfig.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BSP_USING_ADC1
#ifndef ADC1_CONFIG
#define ADC1_CONFIG                                                 \
    {                                                               \
       .Instance                   = hwp_gpadc1,                          \
       .Init.atten3        = 0,             \
       .Init.adc_se            = 1,            \
       .Init.adc_force_on             = 0,           \
       .Init.dma_en          = 0,    \
       .Init.op_mode          = 0,           \
       .Init.en_slot      = 0,                       \
       .Init.data_samp_delay        = 2,          \
       .Init.conv_width        = 75,          \
       .Init.sample_width        = 71,          \
    }
#endif /* ADC1_CONFIG */

#ifdef BSP_ADC1_USING_DMA
#ifndef ADC1_DMA_CONFIG
#define ADC1_DMA_CONFIG                               \
    {                                                 \
        .dma_irq_prio = GPADC_IRQ_PRIO,               \
        .Instance = GPADC_DMA_INSTANCE,               \
        .dma_irq = GPADC_DMA_IRQ,                     \
        .request = GPADC_DMA_REQUEST,                 \
    }

#endif  /* ADC_DMA_CONFIG */
#endif  /* BSP_ADC_USING_DMA */

#endif /* BSP_USING_ADC1 */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_CONFIG_H__ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
