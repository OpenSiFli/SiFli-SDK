/**
  ******************************************************************************
  * @file   audio_config.h
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
       .dma_request             = MIC_DMA_REQUEST,  \
       .is_record               = 1,            \
       .i2s_handle              = hwp_i2s1,      \
       .reqdma_tx               = I2S_TX_DMA_REQUEST, \
       .hdma_tx                 = I2S_TX_DMA_INSTANCE, \
    }
#endif /* BF0_MIC_CONFIG */
#endif /* BSP_ENABLE_I2S_MIC */

#ifdef BSP_ENABLE_I2S_CODEC
#ifndef BF0_I2S2_CONFIG
#define BF0_I2S2_CONFIG \
    {                                           \
       .name                    = "i2s2",       \
       .dma_handle              = MIC_DMA_INSTANCE, \
       .dma_request             = MIC_DMA_REQUEST,  \
       .is_record               = 1,            \
       .i2s_handle              = hwp_i2s2,      \
       .reqdma_tx               = I2S_TX_DMA_REQUEST, \
       .hdma_tx                 = I2S_TX_DMA_INSTANCE, \
    }
#endif /* BF0_I2S2_CONFIG */
#endif /* BSP_ENABLE_I2S_CODEC */

#endif /*  BSP_USING_I2S */


#ifdef __cplusplus
}
#endif

#endif /* __AUDIO_CONFIG_H__ */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
