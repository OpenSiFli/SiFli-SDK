/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __AUDIO_CONFIG_H__
#define __AUDIO_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BSP_USING_I2S

#ifndef BF0_I2S1_CONFIG
#define BF0_I2S1_CONFIG \
    {                                           \
       .name                    = "i2s1",       \
       .dma_handle              = I2S1_RX_DMA_INSTANCE, \
       .dma_request             = I2S1_RX_DMA_REQUEST,  \
       .dma_irq                 = I2S1_RX_DMA_IRQ, \
       .is_record               = 1,            \
       .i2s_handle              = hwp_i2s1,      \
       .reqdma_tx               = I2S1_TX_DMA_REQUEST, \
       .hdma_tx                 = I2S1_TX_DMA_INSTANCE, \
       .tx_dma_irq              = I2S1_TX_DMA_IRQ, \
    }
#endif /* BF0_I2S1_CONFIG */

#ifndef BF0_I2S2_CONFIG
#define BF0_I2S2_CONFIG \
    {                                           \
       .name                    = "i2s2",       \
       .dma_handle              = I2S2_RX_DMA_INSTANCE, \
       .dma_request             = I2S2_RX_DMA_REQUEST,  \
       .dma_irq                 = I2S2_RX_DMA_IRQ, \
       .is_record               = 1,            \
       .i2s_handle              = hwp_i2s2,      \
       .reqdma_tx               = I2S2_TX_DMA_REQUEST, \
       .hdma_tx                 = I2S2_TX_DMA_INSTANCE, \
       .tx_dma_irq              = I2S2_TX_DMA_IRQ, \
    }
#endif /* BF0_I2S2_CONFIG */

#endif /*  BSP_USING_I2S */


#ifdef __cplusplus
}
#endif

#endif /* __AUDIO_CONFIG_H__ */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
