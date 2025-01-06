/**
  ******************************************************************************
  * @file   dma_config.h
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

#ifndef __DMA_CONFIG_H__
#define __DMA_CONFIG_H__

#include <rtconfig.h>

#ifdef __cplusplus
extern "C" {
#endif

/*************************************DMA1 ***************************************/

/* DMA1 channel1 */
#if defined(BSP_QSPI1_USING_DMA) && !defined(FLASH1_DMA_INSTANCE)
#define FLASH1_IRQHandler              DMAC1_CH1_IRQHandler
#define FLASH1_DMA_IRQ_PRIO            0
#define FLASH1_DMA_INSTANCE            DMA1_Channel1
#define FLASH1_DMA_REQUEST             DMA_REQUEST_0
#define FLASH1_DMA_IRQ                 DMAC1_CH1_IRQn
#endif

/* DMA1 channel2 */
#if defined(BSP_QSPI2_USING_DMA) && !defined(FLASH2_DMA_INSTANCE)
#define FLASH2_IRQHandler              DMAC1_CH2_IRQHandler
#define FLASH2_DMA_IRQ_PRIO            0
#define FLASH2_DMA_INSTANCE            DMA1_Channel2
#define FLASH2_DMA_REQUEST             DMA_REQUEST_1
#define FLASH2_DMA_IRQ                 DMAC1_CH2_IRQn
#endif

/* DMA1 channel3 */
#if defined(BSP_QSPI3_USING_DMA) && !defined(FLASH3_DMA_INSTANCE)
#define FLASH3_IRQHandler              DMAC1_CH3_IRQHandler
#define FLASH3_DMA_IRQ_PRIO            0
#define FLASH3_DMA_INSTANCE            DMA1_Channel3
#define FLASH3_DMA_REQUEST             DMA_REQUEST_2
#define FLASH3_DMA_IRQ                 DMAC1_CH3_IRQn
#endif

/* DMA1 channel3 */
//TODO: need to be removed, not used by A0
#if defined(RT_USING_USB_DEVICE) && !defined(USBD_TX_DMA_INSTANCE)
#define USBD_RX_IRQHandler              DMAC1_CH3_IRQHandler
#define USBD_RX_DMA_IRQ_PRIO            0
#define USBD_RX_DMA_INSTANCE            DMA1_Channel3
#define USBD_RX_DMA_REQUEST             DMA_REQUEST_2
#define USBD_RX_DMA_IRQ                 DMAC1_CH3_IRQn
#endif
//PDM1
#if !defined(PDM1_L_DMA_INSTANCE)
#define PDM1_L_DMA_IRQHandler           DMAC1_CH3_IRQHandler
#define PDM1_L_DMA_IRQ_PRIO             0
#define PDM1_L_DMA_INSTANCE             DMA1_Channel3
#define PDM1_L_DMA_REQUEST              DMA_REQUEST_18
#define PDM1_L_DMA_IRQ                  DMAC1_CH3_IRQn
#endif

/* DMA1 channel4 */
//TODO: need to be removed, not used by A0
#if defined(RT_USING_USB_DEVICE) && !defined(USBD_TX_DMA_INSTANCE)
#define USBD_TX_IRQHandler              DMAC1_CH4_IRQHandler
#define USBD_TX_DMA_IRQ_PRIO            0
#define USBD_TX_DMA_INSTANCE            DMA1_Channel4
#define USBD_TX_DMA_REQUEST             DMA_REQUEST_3
#define USBD_TX_DMA_IRQ                 DMAC1_CH4_IRQn
#endif
//PDM1
#if !defined(PDM1_R_DMA_INSTANCE)
#define PDM1_R_DMA_IRQHandler           DMAC1_CH4_IRQHandler
#define PDM1_R_DMA_IRQ_PRIO             0
#define PDM1_R_DMA_INSTANCE             DMA1_Channel4
#define PDM1_R_DMA_REQUEST              DMA_REQUEST_19
#define PDM1_R_DMA_IRQ                  DMAC1_CH4_IRQn
#endif

/* DMA1 channel5 */
#if defined(BSP_UART1_TX_USING_DMA) && !defined(UART1_TX_DMA_INSTANCE)
#define UART1_DMA_TX_IRQHandler          DMAC1_CH5_IRQHandler
#define UART1_TX_DMA_IRQ_PRIO            0
#define UART1_TX_DMA_INSTANCE            DMA1_Channel5
#define UART1_TX_DMA_REQUEST             DMA_REQUEST_4
#define UART1_TX_DMA_IRQ                 DMAC1_CH5_IRQn
#endif

/*UART 2 RX DMA, shared with UART 1 TX DMA*/
#if defined(BSP_UART2_RX_USING_DMA) && !defined(UART2_RX_DMA_INSTANCE)
#define UART2_DMA_RX_IRQHandler          DMAC1_CH5_IRQHandler
#define UART2_RX_DMA_IRQ_PRIO            0
#define UART2_RX_DMA_INSTANCE            DMA1_Channel5
#define UART2_RX_DMA_REQUEST             DMA_REQUEST_7
#define UART2_RX_DMA_IRQ                 DMAC1_CH5_IRQn
#endif


/* DMA1 channel6 */
#if defined(BSP_UART1_RX_USING_DMA) && !defined(UART1_RX_DMA_INSTANCE)
#define UART1_DMA_RX_IRQHandler          DMAC1_CH6_IRQHandler
#define UART1_RX_DMA_IRQ_PRIO            0
#define UART1_RX_DMA_INSTANCE            DMA1_Channel6
#define UART1_RX_DMA_REQUEST             DMA_REQUEST_5
#define UART1_RX_DMA_IRQ                 DMAC1_CH6_IRQn
#endif

/* DMA1 channel7  */
#ifdef BSP_ENABLE_I2S_MIC
//I2S1
#if defined(BSP_USING_I2S) && !defined(MIC_DMA_INSTANCE)
#define MIC_DMA_RX_IRQHandler           DMAC1_CH7_IRQHandler
#define MIC_DMA_IRQ_PRIO                0
#define MIC_DMA_INSTANCE                DMA1_Channel7
#define MIC_DMA_REQUEST                 DMA_REQUEST_19
#define MIC_DMA_IRQ                     DMAC1_CH7_IRQn
#endif

/* DMA1 channel8  */
//I2S1
#if defined(BSP_USING_I2S) && !defined(I2S_TX_DMA_INSTANCE)
#define I2S_TX_DMA_IRQHandler           DMAC1_CH8_IRQHandler
#define I2S_TX_DMA_IRQ_PRIO             0
#define I2S_TX_DMA_INSTANCE             DMA1_Channel8
#define I2S_TX_DMA_REQUEST              DMA_REQUEST_18
#define I2S_TX_DMA_IRQ                  DMAC1_CH8_IRQn
#endif

#elif defined (BSP_ENABLE_I2S_CODEC)
//I2S2 RX,
#ifndef BSP_USING_PDM2
#define MIC_DMA_RX_IRQHandler           DMAC1_CH7_IRQHandler
#define MIC_DMA_IRQ_PRIO                0
#define MIC_DMA_INSTANCE                DMA1_Channel7
#define MIC_DMA_REQUEST                 DMA_REQUEST_21
#define MIC_DMA_IRQ                     DMAC1_CH7_IRQn
#else // When enable PDM2, it's dma request same to I2S2 RX, so can not use I2S2 RX any more.
//#define MIC_DMA_RX_IRQHandler           DMAC1_CH7_IRQHandler
#define MIC_DMA_IRQ_PRIO                0
#define MIC_DMA_INSTANCE                DMA1_Channel7
#define MIC_DMA_REQUEST                 0xff
#define MIC_DMA_IRQ                     DMAC1_CH7_IRQn

#endif //BSP_USING_PDM2
/* DMA1 channel8  */
//I2S2
#if defined(BSP_USING_I2S) && !defined(I2S_TX_DMA_INSTANCE)
#define I2S_TX_DMA_IRQHandler           DMAC1_CH8_IRQHandler
#define I2S_TX_DMA_IRQ_PRIO             0
#define I2S_TX_DMA_INSTANCE             DMA1_Channel8
#define I2S_TX_DMA_REQUEST              DMA_REQUEST_20
#define I2S_TX_DMA_IRQ                  DMAC1_CH8_IRQn
#endif

#endif

//PDM2
#if defined(BSP_USING_PDM2) && !defined(PDM2_R_DMA_INSTANCE)
#define PDM2_R_DMA_IRQHandler           DMAC1_CH7_IRQHandler
#define PDM2_R_DMA_IRQ_PRIO             0
#define PDM2_R_DMA_INSTANCE             DMA1_Channel7
#define PDM2_R_DMA_REQUEST              DMA_REQUEST_21
#define PDM2_R_DMA_IRQ                  DMAC1_CH7_IRQn
#endif

/*************************************DMA2 ***************************************/
/* DMA2 channel1  */
#if defined(BSP_UART3_TX_USING_DMA) && !defined(UART3_TX_DMA_INSTANCE)
#define UART3_DMA_TX_IRQHandler         DMAC2_CH1_IRQHandler
#define UART3_TX_DMA_IRQ_PRIO           0
#define UART3_TX_DMA_INSTANCE           DMA2_Channel1
#define UART3_TX_DMA_REQUEST            DMA_REQUEST_0
#define UART3_TX_DMA_IRQ                DMAC2_CH1_IRQn
#endif

/* DMA2 channel2  */
#if defined(BSP_UART3_RX_USING_DMA) && !defined(UART3_RX_DMA_INSTANCE)
#define UART3_DMA_RX_IRQHandler         DMAC2_CH2_IRQHandler
#define UART3_RX_DMA_IRQ_PRIO           0
#define UART3_RX_DMA_INSTANCE           DMA2_Channel2
#define UART3_RX_DMA_REQUEST            DMA_REQUEST_1
#define UART3_RX_DMA_IRQ                DMAC2_CH2_IRQn
#endif


/* DMA1 channel4  */
#if defined(BSP_SPI1_RX_USING_DMA) && !defined(SPI1_RX_DMA_INSTANCE) && defined(BSP_USING_SPI_CAMERA)
#define SPI1_DMA_RX_IRQHandler         DMAC1_CH4_IRQHandler
#define SPI1_RX_DMA_IRQ_PRIO           0
#define SPI1_RX_DMA_INSTANCE           DMA1_Channel4
#define SPI1_RX_DMA_REQUEST            DMA_REQUEST_29
#define SPI1_RX_DMA_IRQ                DMAC1_CH4_IRQn
#endif


/* DMA2 channel3  */
#if defined(BSP_SPI3_RX_USING_DMA) && !defined(SPI3_RX_DMA_INSTANCE)
#define SPI3_DMA_RX_IRQHandler         DMAC2_CH3_IRQHandler
#define SPI3_RX_DMA_IRQ_PRIO           0
#define SPI3_RX_DMA_INSTANCE           DMA2_Channel3
#define SPI3_RX_DMA_REQUEST            DMA_REQUEST_17
#define SPI3_RX_DMA_IRQ                DMAC2_CH3_IRQn
#endif

/* DMA2 channel4  */
#if defined(BSP_SPI3_TX_USING_DMA) && !defined(SPI3_TX_DMA_INSTANCE)
#define SPI3_DMA_TX_IRQHandler         DMAC2_CH4_IRQHandler
#define SPI3_TX_DMA_IRQ_PRIO           0
#define SPI3_TX_DMA_INSTANCE           DMA2_Channel4
#define SPI3_TX_DMA_REQUEST            DMA_REQUEST_16
#define SPI3_TX_DMA_IRQ                DMAC2_CH4_IRQn
#endif

/* DMA2 channel3 */
#if defined(BSP_I2C4_USING_DMA) && !defined(I2C4_DMA_INSTANCE)
#define I2C4_DMA_IRQHandler              DMAC2_CH3_IRQHandler
#define I2C4_DMA_IRQ_PRIO                0
#define I2C4_DMA_INSTANCE                DMA2_Channel3
#define I2C4_DMA_REQUEST                 DMA_REQUEST_21
#define I2C4_DMA_IRQ                     DMAC2_CH3_IRQn
#endif

/* DMA2 channel4 */
#if defined(BSP_I2C5_USING_DMA) && !defined(I2C5_DMA_INSTANCE)
#define I2C5_DMA_IRQHandler              DMAC2_CH4_IRQHandler
#define I2C5_DMA_IRQ_PRIO                0
#define I2C5_DMA_INSTANCE                DMA2_Channel4
#define I2C5_DMA_REQUEST                 DMA_REQUEST_22
#define I2C5_DMA_IRQ                     DMAC2_CH4_IRQn
#endif

/* DMA2 channel5 */
#if defined(BSP_UART4_TX_USING_DMA) && !defined(UART4_TX_DMA_INSTANCE)
#define UART4_DMA_TX_IRQHandler          DMAC2_CH5_IRQHandler
#define UART4_TX_DMA_IRQ_PRIO            0
#define UART4_TX_DMA_INSTANCE            DMA2_Channel5
#define UART4_TX_DMA_REQUEST             DMA_REQUEST_2
#define UART4_TX_DMA_IRQ                 DMAC2_CH5_IRQn
#endif

/* DMA2 channel5 */
#if defined(BSP_GPADC_USING_DMA) && !defined(GPADC_DMA_INSTANCE)
#define GPADC_IRQHandler              DMAC2_CH5_IRQHandler
#define GPADC_DMA_IRQ_PRIO            0
#define GPADC_DMA_INSTANCE            DMA2_Channel5
#define GPADC_DMA_REQUEST             DMA_REQUEST_30
#define GPADC_DMA_IRQ                 DMAC2_CH5_IRQn
#endif

/* DMA2 channel5 */
#if defined(BSP_SDADC_USING_DMA) && !defined(SDADC_DMA_INSTANCE)
#define SDADC_IRQHandler              DMAC2_CH5_IRQHandler
#define SDADC_DMA_IRQ_PRIO            0
#define SDADC_DMA_INSTANCE            DMA2_Channel5
#define SDADC_DMA_REQUEST             DMA_REQUEST_31
#define SDADC_DMA_IRQ                 DMAC2_CH5_IRQn
#endif

/* DMA2 channel6 */
#if defined(BSP_UART4_RX_USING_DMA) && !defined(UART4_RX_DMA_INSTANCE)
#define UART4_DMA_RX_IRQHandler          DMAC2_CH6_IRQHandler
#define UART4_RX_DMA_IRQ_PRIO            0
#define UART4_RX_DMA_INSTANCE            DMA2_Channel6
#define UART4_RX_DMA_REQUEST             DMA_REQUEST_3
#define UART4_RX_DMA_IRQ                 DMAC2_CH6_IRQn
#endif

/* DMA2 channel7 */
#if defined(BSP_SPI4_RX_USING_DMA) && !defined(SPI4_RX_DMA_INSTANCE)
#define SPI4_DMA_RX_IRQHandler         DMAC2_CH7_IRQHandler
#define SPI4_RX_DMA_IRQ_PRIO           0
#define SPI4_RX_DMA_INSTANCE           DMA2_Channel7
#define SPI4_RX_DMA_REQUEST            DMA_REQUEST_19
#define SPI4_RX_DMA_IRQ                DMAC2_CH7_IRQn
#endif

/* DMA2 channel8 */
#if defined(BSP_I2C6_USING_DMA) && !defined(I2C6_DMA_INSTANCE)
#define I2C6_DMA_IRQHandler              DMAC2_CH8_IRQHandler
#define I2C6_DMA_IRQ_PRIO                0
#define I2C6_DMA_INSTANCE                DMA2_Channel8
#define I2C6_DMA_REQUEST                 DMA_REQUEST_23
#define I2C6_DMA_IRQ                     DMAC2_CH8_IRQn
#endif

/* DMA2 channel8 */
#if defined(BSP_SPI4_TX_USING_DMA) && !defined(SPI4_TX_DMA_INSTANCE)
#define SPI4_DMA_TX_IRQHandler         DMAC2_CH8_IRQHandler
#define SPI4_TX_DMA_IRQ_PRIO           0
#define SPI4_TX_DMA_INSTANCE           DMA2_Channel8
#define SPI4_TX_DMA_REQUEST            DMA_REQUEST_18
#define SPI4_TX_DMA_IRQ                DMAC2_CH8_IRQn
#endif

/* DMA2 channel7 */
#if defined(BSP_UART5_TX_USING_DMA) && !defined(UART5_TX_DMA_INSTANCE)
#define UART5_DMA_TX_IRQHandler          DMAC2_CH7_IRQHandler
#define UART5_TX_DMA_IRQ_PRIO            0
#define UART5_TX_DMA_INSTANCE            DMA2_Channel7
#define UART5_TX_DMA_REQUEST             DMA_REQUEST_4
#define UART5_TX_DMA_IRQ                 DMAC2_CH7_IRQn
#endif


/* DMA2 channel8 */
#if defined(BSP_UART5_RX_USING_DMA) && !defined(UART5_RX_DMA_INSTANCE)
#define UART5_DMA_RX_IRQHandler          DMAC2_CH8_IRQHandler
#define UART5_RX_DMA_IRQ_PRIO            0
#define UART5_RX_DMA_INSTANCE            DMA2_Channel8
#define UART5_RX_DMA_REQUEST             DMA_REQUEST_5
#define UART5_RX_DMA_IRQ                 DMAC2_CH8_IRQn
#endif


/* DMA2 channel8 */
#if defined(BSP_QSPI4_USING_DMA) && !defined(FLASH4_DMA_INSTANCE)
#define FLASH4_IRQHandler              DMAC2_CH8_IRQHandler
#define FLASH4_DMA_IRQ_PRIO            0
#define FLASH4_DMA_INSTANCE            DMA2_Channel8
#define FLASH4_DMA_REQUEST             DMA_REQUEST_20
#define FLASH4_DMA_IRQ                 DMAC2_CH8_IRQn
#endif

#ifdef __cplusplus
}
#endif

#endif /* __DMA_CONFIG_H__ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
