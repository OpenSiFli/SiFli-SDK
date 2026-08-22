/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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
       .Instance               = hwp_gpadc1,                        \
       .Init.atten3            = 0,                                 \
       .Init.adc_se            = 1,                                 \
       .Init.adc_force_on      = 0,                                 \
       .Init.dma_en            = 0,                                 \
       .Init.op_mode           = 0,                                 \
       .Init.en_slot           = 0,                                 \
       .Init.data_samp_delay   = 3,                                 \
       .Init.conv_width        = 100,                               \
       .Init.sample_width      = 95,                                \
    }
#endif /* ADC1_CONFIG */

#ifdef BSP_ADC1_USING_DMA
#ifndef ADC1_DMA_CONFIG
#define ADC1_DMA_CONFIG                               \
    {                                                 \
        .dma_rcc = GPADC_DMA_RCC,                     \
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
