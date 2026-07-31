/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __AUDIO_CONFIG_H__
#define __AUDIO_CONFIG_H__

#include <rtconfig.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BSP_USING_I2S
#ifdef BSP_ENABLE_I2S_MIC
#ifndef BF0_MIC_CONFIG
#define BF0_MIC_CONFIG \
    {                                           \
       .name                    = "i2s1",       \
       .dma_handle              = MIC_DMA_INSTANCE, \
       .dma_request             = I2S1_RX_DMA_REQUEST,  \
       .is_record               = 1,            \
       .i2s_handle              = hwp_i2s1,      \
       .reqdma_tx               = I2S1_TX_DMA_REQUEST, \
       .hdma_tx                 = MIC_TX_DMA_INSTANCE, \
    }
#endif /* BF0_MIC_CONFIG */
#endif /* BSP_ENABLE_I2S_MIC */

#ifdef BSP_ENABLE_I2S_CODEC
#ifndef BF0_I2S2_CONFIG
#define BF0_I2S2_CONFIG \
    {                                           \
       .name                    = "i2s2",       \
       .dma_handle              = I2S_RX_DMA_INSTANCE, \
       .dma_request             = I2S2_RX_DMA_REQUEST,  \
       .dma_irq                 = I2S_RX_DMA_IRQ, \
       .is_record               = 1,            \
       .i2s_handle              = hwp_i2s2,      \
       .reqdma_tx               = I2S2_TX_DMA_REQUEST, \
       .hdma_tx                 = I2S_TX_DMA_INSTANCE, \
       .tx_dma_irq              = I2S_TX_DMA_IRQ, \
    }
#endif /* BF0_I2S2_CONFIG */
#endif /* BSP_ENABLE_I2S_CODEC */

#ifdef BSP_ENABLE_I2S3
#ifndef BF0_I2S3_CONFIG
#define BF0_I2S3_CONFIG \
    {                                           \
       .name                    = "i2s3",       \
       .dma_handle              = I2S3_RX_DMA_INSTANCE, \
       .dma_request             = I2S3_RX_DMA_REQUEST,  \
       .dma_irq                 = I2S3_RX_DMA_IRQ, \
       .is_record               = 1,            \
       .i2s_handle              = hwp_i2s3,      \
       .reqdma_tx               = I2S3_TX_DMA_REQUEST, \
       .hdma_tx                 = I2S3_TX_DMA_INSTANCE, \
       .tx_dma_irq              = I2S3_TX_DMA_IRQ, \
    }
#endif /* BF0_I2S2_CONFIG */
#endif /* BSP_ENABLE_I2S_CODEC */

#endif /*  BSP_USING_I2S */

#ifdef __cplusplus
}
#endif

#endif /* __AUDIO_CONFIG_H__ */

