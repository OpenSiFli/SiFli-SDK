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
#if defined(BSP_I2C3_USING_DMA) && !defined(I2C3_DMA_INSTANCE)
#define I2C3_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define I2C3_DMA_IRQ_PRIO                1
#define I2C3_DMA_INSTANCE                DMA1_Channel2
#define I2C3_DMA_REQUEST                 DMA_REQUEST_24
#define I2C3_DMA_IRQ                     DMAC1_CH2_IRQn
#endif
#if defined(BSP_I2C4_USING_DMA) && !defined(I2C4_DMA_INSTANCE)
#define I2C4_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define I2C4_DMA_IRQ_PRIO                1
#define I2C4_DMA_INSTANCE                DMA1_Channel2
#define I2C4_DMA_REQUEST                 DMA_REQUEST_1 //Reuse with MPI2's request
#define I2C4_DMA_IRQ                     DMAC1_CH2_IRQn
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
#if defined(BSP_QSPI4_USING_DMA) && !defined(FLASH4_DMA_INSTANCE)
#define FLASH4_IRQHandler              DMAC1_CH4_IRQHandler
#define FLASH4_DMA_IRQ_PRIO            0
#define FLASH4_DMA_INSTANCE            DMA1_Channel4
#define FLASH4_DMA_REQUEST             DMA_REQUEST_3
#define FLASH4_DMA_IRQ                 DMAC1_CH4_IRQn
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
#if defined(BSP_SPI1_RX_USING_DMA) && !defined(SPI1_RX_DMA_INSTANCE)
#define SPI1_DMA_RX_IRQHandler         DMAC1_CH3_IRQHandler
#define SPI1_RX_DMA_IRQ_PRIO           0
#define SPI1_RX_DMA_INSTANCE           DMA1_Channel3
#define SPI1_RX_DMA_REQUEST            DMA_REQUEST_29
#define SPI1_RX_DMA_IRQ                DMAC1_CH3_IRQn
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
#if defined(BSP_SPI1_TX_USING_DMA) && !defined(SPI1_TX_DMA_INSTANCE)
#define SPI1_DMA_TX_IRQHandler         DMAC1_CH4_IRQHandler
#define SPI1_TX_DMA_IRQ_PRIO           0
#define SPI1_TX_DMA_INSTANCE           DMA1_Channel4
#define SPI1_TX_DMA_REQUEST            DMA_REQUEST_28
#define SPI1_TX_DMA_IRQ                DMAC1_CH4_IRQn
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
#if defined(BSP_UART3_TX_USING_DMA) && !defined(UART3_TX_DMA_INSTANCE)
#define UART3_DMA_TX_IRQHandler         DMAC1_CH7_IRQHandler
#define UART3_TX_DMA_IRQ_PRIO           0
#define UART3_TX_DMA_INSTANCE           DMA1_Channel7
#define UART3_TX_DMA_REQUEST            DMA_REQUEST_26
#define UART3_TX_DMA_IRQ                DMAC1_CH7_IRQn
#endif
#if defined(BSP_SPI2_RX_USING_DMA) && !defined(SPI2_RX_DMA_INSTANCE)
#define SPI2_DMA_RX_IRQHandler         DMAC1_CH7_IRQHandler
#define SPI2_RX_DMA_IRQ_PRIO           0
#define SPI2_RX_DMA_INSTANCE           DMA1_Channel7
#define SPI2_RX_DMA_REQUEST            DMA_REQUEST_31
#define SPI2_RX_DMA_IRQ                DMAC1_CH7_IRQn
#endif


/* DMA1 channel8  */
#if defined(BSP_UART3_RX_USING_DMA) && !defined(UART3_RX_DMA_INSTANCE)
#define UART3_DMA_RX_IRQHandler         DMAC1_CH8_IRQHandler
#define UART3_RX_DMA_IRQ_PRIO           0
#define UART3_RX_DMA_INSTANCE           DMA1_Channel8
#define UART3_RX_DMA_REQUEST            DMA_REQUEST_27
#define UART3_RX_DMA_IRQ                DMAC1_CH8_IRQn
#endif
#if defined(BSP_SPI2_TX_USING_DMA) && !defined(SPI2_TX_DMA_INSTANCE)
#define SPI2_DMA_TX_IRQHandler         DMAC1_CH8_IRQHandler
#define SPI2_TX_DMA_IRQ_PRIO           0
#define SPI2_TX_DMA_INSTANCE           DMA1_Channel8
#define SPI2_TX_DMA_REQUEST            DMA_REQUEST_30
#define SPI2_TX_DMA_IRQ                DMAC1_CH8_IRQn
#endif


/*************************************DMA2 ***************************************/

/* DMA2 channel1  */

//I2S1 TX
#if defined(BSP_ENABLE_I2S_MIC) && !defined(MIC_TX_DMA_INSTANCE)
#define MIC_TX_DMA_IRQHandler              DMAC2_CH1_IRQHandler
#define MIC_TX_DMA_IRQ_PRIO                0
#define MIC_TX_DMA_INSTANCE                DMA2_Channel1
#define MIC_TX_DMA_REQUEST                 DMA_REQUEST_0
#define MIC_TX_DMA_IRQ                     DMAC2_CH1_IRQn
#endif

// AUDPRC TX CH0
#if defined(BSP_AUDPRC_TX0_DMA) && !defined(AUDPRC_TX0_DMA_INSTANCE)
#define AUDPRC_TX0_DMA_IRQHandler              DMAC2_CH1_IRQHandler
#define AUDPRC_TX0_DMA_IRQ_PRIO                0
#define AUDPRC_TX0_DMA_INSTANCE                DMA2_Channel1
#define AUDPRC_TX0_DMA_REQUEST                 DMA_REQUEST_19
#define AUDPRC_TX0_DMA_IRQ                     DMAC2_CH1_IRQn
#endif

// AUDCODEC DAC CH0
#if defined(BSP_AUDCODEC_DAC0_DMA) && !defined(AUDCODEC_DAC0_DMA_INSTANCE)
#define AUDCODEC_DAC0_DMA_IRQHandler              DMAC2_CH1_IRQHandler
#define AUDCODEC_DAC0_DMA_IRQ_PRIO                0
#define AUDCODEC_DAC0_DMA_INSTANCE                DMA2_Channel1
#define AUDCODEC_DAC0_DMA_REQUEST                 DMA_REQUEST_9
#define AUDCODEC_DAC0_DMA_IRQ                     DMAC2_CH1_IRQn
#endif


/* DMA2 channel2  */

//I2S1 RX
#if defined(BSP_ENABLE_I2S_MIC) && !defined(MIC_DMA_INSTANCE)
#define MIC_DMA_RX_IRQHandler           DMAC2_CH2_IRQHandler
#define MIC_DMA_IRQ_PRIO                0
#define MIC_DMA_INSTANCE                DMA2_Channel2
#define MIC_DMA_REQUEST                 DMA_REQUEST_1
#define MIC_DMA_IRQ                     DMAC2_CH2_IRQn
#endif

// AUDPRC TX CH1
#if defined(BSP_AUDPRC_TX1_DMA) && !defined(AUDPRC_TX1_DMA_INSTANCE)
#define AUDPRC_TX1_DMA_IRQHandler              DMAC2_CH2_IRQHandler
#define AUDPRC_TX1_DMA_IRQ_PRIO                0
#define AUDPRC_TX1_DMA_INSTANCE                DMA2_Channel2
#define AUDPRC_TX1_DMA_REQUEST                 DMA_REQUEST_18
#define AUDPRC_TX1_DMA_IRQ                     DMAC2_CH2_IRQn
#endif

// AUDCODEC DAC CH1
#if defined(BSP_AUDCODEC_DAC1_DMA) && !defined(AUDCODEC_DAC1_DMA_INSTANCE)
#define AUDCODEC_DAC1_DMA_IRQHandler              DMAC2_CH2_IRQHandler
#define AUDCODEC_DAC1_DMA_IRQ_PRIO                0
#define AUDCODEC_DAC1_DMA_INSTANCE                DMA2_Channel2
#define AUDCODEC_DAC1_DMA_REQUEST                 DMA_REQUEST_10
#define AUDCODEC_DAC1_DMA_IRQ                     DMAC2_CH2_IRQn
#endif


/* DMA2 channel3  */
//I2S2 TX
#if defined(BSP_ENABLE_I2S_CODEC) && !defined(I2S_TX_DMA_INSTANCE)
#ifdef SOC_BF0_HCPU
#define I2S_TX_DMA_IRQHandler              DMAC2_CH3_IRQHandler
#define I2S_TX_DMA_IRQ                     DMAC2_CH3_IRQn
#else // LCPU
#define I2S_TX_DMA_IRQHandler              Interrupt30_IRQHandler
#define I2S_TX_DMA_IRQ                     Interrupt30_IRQn
#endif  //SOC_BF0_HCPU
#define I2S_TX_DMA_IRQ_PRIO                0
#define I2S_TX_DMA_INSTANCE                DMA2_Channel3
#define I2S_TX_DMA_REQUEST                 DMA_REQUEST_2
#else
// AUDPRC RX CH1
#if defined(BSP_AUDPRC_RX1_DMA) && !defined(AUDPRC_RX1_DMA_INSTANCE)
#define AUDPRC_RX1_DMA_IRQHandler              DMAC2_CH3_IRQHandler
#define AUDPRC_RX1_DMA_IRQ_PRIO                0
#define AUDPRC_RX1_DMA_INSTANCE                DMA2_Channel3
#define AUDPRC_RX1_DMA_REQUEST                 DMA_REQUEST_20
#define AUDPRC_RX1_DMA_IRQ                     DMAC2_CH3_IRQn
#endif

#endif

/* DMA2 channel4  */
//I2S2 RX
#if defined(BSP_ENABLE_I2S_CODEC) && !defined(I2S_RX_DMA_INSTANCE)
#ifdef SOC_BF0_HCPU
#define I2S_RX_DMA_IRQHandler              DMAC2_CH4_IRQHandler
#define I2S_RX_DMA_IRQ                     DMAC2_CH4_IRQn
#else // LCPU
#define I2S_RX_DMA_IRQHandler              Interrupt31_IRQHandler
#define I2S_RX_DMA_IRQ                     Interrupt31_IRQn
#endif //SOC_BF0_HCPU
#define I2S_RX_DMA_IRQ_PRIO                0
#define I2S_RX_DMA_INSTANCE                DMA2_Channel4
#define I2S_RX_DMA_REQUEST                 DMA_REQUEST_3
#else
// AUDPRC TX OUT CH1
#if defined(BSP_AUDPRC_TX_OUT1_DMA) && !defined(AUDPRC_TX_OUT1_DMA_INSTANCE)
#define AUDPRC_TX_OUT1_DMA_IRQHandler              DMAC2_CH4_IRQHandler
#define AUDPRC_TX_OUT1_DMA_IRQ_PRIO                0
#define AUDPRC_TX_OUT1_DMA_INSTANCE                DMA2_Channel4
#define AUDPRC_TX_OUT1_DMA_REQUEST                 DMA_REQUEST_14
#define AUDPRC_TX_OUT1_DMA_IRQ                     DMAC2_CH4_IRQn
#endif

#endif

/* DMA2 channel5  */
//PDM1 L
#if !defined(PDM1_L_DMA_INSTANCE)
#define PDM1_L_DMA_IRQHandler           DMAC2_CH5_IRQHandler
#define PDM1_L_DMA_IRQ_PRIO             0
#define PDM1_L_DMA_INSTANCE             DMA2_Channel5
#define PDM1_L_DMA_REQUEST              DMA_REQUEST_4
#define PDM1_L_DMA_IRQ                  DMAC2_CH5_IRQn
#endif

// AUDPRC RX CH0
#if defined(BSP_AUDPRC_RX0_DMA) && !defined(AUDPRC_RX0_DMA_INSTANCE)
#define AUDPRC_RX0_DMA_IRQHandler              DMAC2_CH5_IRQHandler
#define AUDPRC_RX0_DMA_IRQ_PRIO                0
#define AUDPRC_RX0_DMA_INSTANCE                DMA2_Channel5
#define AUDPRC_RX0_DMA_REQUEST                 DMA_REQUEST_21
#define AUDPRC_RX0_DMA_IRQ                     DMAC2_CH5_IRQn
#endif

/* DMA2 channel6  */
//PDM1 R
#if !defined(PDM1_R_DMA_INSTANCE)
#define PDM1_R_DMA_IRQHandler              DMAC2_CH6_IRQHandler
#define PDM1_R_DMA_IRQ_PRIO                0
#define PDM1_R_DMA_INSTANCE                DMA2_Channel6
#define PDM1_R_DMA_REQUEST                 DMA_REQUEST_5
#define PDM1_R_DMA_IRQ                     DMAC2_CH6_IRQn
#endif

// AUDPRC TX OUT CH0
#if defined(BSP_AUDPRC_TX_OUT0_DMA) && !defined(AUDPRC_TX_OUT0_DMA_INSTANCE)
#define AUDPRC_TX_OUT0_DMA_IRQHandler              DMAC2_CH6_IRQHandler
#define AUDPRC_TX_OUT0_DMA_IRQ_PRIO                0
#define AUDPRC_TX_OUT0_DMA_INSTANCE                DMA2_Channel6
#define AUDPRC_TX_OUT0_DMA_REQUEST                 DMA_REQUEST_15
#define AUDPRC_TX_OUT0_DMA_IRQ                     DMAC2_CH6_IRQn
#endif


/* DMA2 channel7  */
//PDM2 L
#if !defined(PDM2_L_DMA_INSTANCE)
#define PDM2_L_DMA_IRQHandler           DMAC2_CH7_IRQHandler
#define PDM2_L_DMA_IRQ_PRIO             0
#define PDM2_L_DMA_INSTANCE             DMA2_Channel7
#define PDM2_L_DMA_REQUEST              DMA_REQUEST_6
#define PDM2_L_DMA_IRQ                  DMAC2_CH7_IRQn
#endif

// AUDPRC TX CH2
#if defined(BSP_AUDPRC_TX2_DMA) && !defined(AUDPRC_TX2_DMA_INSTANCE)
#define AUDPRC_TX2_DMA_IRQHandler              DMAC2_CH7_IRQHandler
#define AUDPRC_TX2_DMA_IRQ_PRIO                0
#define AUDPRC_TX2_DMA_INSTANCE                DMA2_Channel7
#define AUDPRC_TX2_DMA_REQUEST                 DMA_REQUEST_17
#define AUDPRC_TX2_DMA_IRQ                     DMAC2_CH7_IRQn
#endif



/* DMA2 channel8  */
//PDM2 R
#if !defined(PDM2_R_DMA_INSTANCE)
#define PDM2_R_DMA_IRQHandler              DMAC2_CH8_IRQHandler
#define PDM2_R_DMA_IRQ_PRIO                0
#define PDM2_R_DMA_INSTANCE                DMA2_Channel8
#define PDM2_R_DMA_REQUEST                 DMA_REQUEST_7
#define PDM2_R_DMA_IRQ                     DMAC2_CH8_IRQn
#endif

// AUDPRC TX CH3
#if defined(BSP_AUDPRC_TX3_DMA) && !defined(AUDPRC_TX3_DMA_INSTANCE)
#define AUDPRC_TX3_DMA_IRQHandler              DMAC2_CH8_IRQHandler
#define AUDPRC_TX3_DMA_IRQ_PRIO                0
#define AUDPRC_TX3_DMA_INSTANCE                DMA2_Channel8
#define AUDPRC_TX3_DMA_REQUEST                 DMA_REQUEST_16
#define AUDPRC_TX3_DMA_IRQ                     DMAC2_CH8_IRQn
#endif



/*************************************DMA3 ***************************************/
/* DMA3 channel1  */
#if defined(BSP_UART4_TX_USING_DMA) && !defined(UART4_TX_DMA_INSTANCE)
#define UART4_DMA_TX_IRQHandler         DMAC3_CH1_IRQHandler
#define UART4_TX_DMA_IRQ_PRIO           0
#define UART4_TX_DMA_INSTANCE           DMA3_Channel1
#define UART4_TX_DMA_REQUEST            DMA_REQUEST_0
#define UART4_TX_DMA_IRQ                DMAC3_CH1_IRQn
#endif

/* DMA3 channel2  */
#if defined(BSP_UART4_RX_USING_DMA) && !defined(UART4_RX_DMA_INSTANCE)
#define UART4_DMA_RX_IRQHandler         DMAC3_CH2_IRQHandler
#define UART4_RX_DMA_IRQ_PRIO           0
#define UART4_RX_DMA_INSTANCE           DMA3_Channel2
#define UART4_RX_DMA_REQUEST            DMA_REQUEST_1
#define UART4_RX_DMA_IRQ                DMAC3_CH2_IRQn
#endif


/* DMA3 channel3  */
#if defined(BSP_SPI3_RX_USING_DMA) && !defined(SPI3_RX_DMA_INSTANCE)
#define SPI3_DMA_RX_IRQHandler         DMAC3_CH3_IRQHandler
#define SPI3_RX_DMA_IRQ_PRIO           0
#define SPI3_RX_DMA_INSTANCE           DMA3_Channel3
#define SPI3_RX_DMA_REQUEST            DMA_REQUEST_17
#define SPI3_RX_DMA_IRQ                DMAC3_CH3_IRQn
#endif

/* DMA3 channel4  */
#if defined(BSP_SPI3_TX_USING_DMA) && !defined(SPI3_TX_DMA_INSTANCE)
#define SPI3_DMA_TX_IRQHandler         DMAC3_CH4_IRQHandler
#define SPI3_TX_DMA_IRQ_PRIO           0
#define SPI3_TX_DMA_INSTANCE           DMA3_Channel4
#define SPI3_TX_DMA_REQUEST            DMA_REQUEST_16
#define SPI3_TX_DMA_IRQ                DMAC3_CH4_IRQn
#endif

/* DMA3 channel3 */
#if defined(BSP_I2C5_USING_DMA) && !defined(I2C5_DMA_INSTANCE)
#define I2C5_DMA_IRQHandler              DMAC3_CH3_IRQHandler
#define I2C5_DMA_IRQ_PRIO                1
#define I2C5_DMA_INSTANCE                DMA3_Channel3
#define I2C5_DMA_REQUEST                 DMA_REQUEST_21
#define I2C5_DMA_IRQ                     DMAC3_CH3_IRQn
#endif

/* DMA3 channel4 */
#if defined(BSP_I2C6_USING_DMA) && !defined(I2C6_DMA_INSTANCE)
#define I2C6_DMA_IRQHandler              DMAC3_CH4_IRQHandler
#define I2C6_DMA_IRQ_PRIO                1
#define I2C6_DMA_INSTANCE                DMA3_Channel4
#define I2C6_DMA_REQUEST                 DMA_REQUEST_22
#define I2C6_DMA_IRQ                     DMAC3_CH4_IRQn
#endif

//I2S3 RX
#if defined(BSP_ENABLE_I2S3) && !defined(I2S3_RX_DMA_INSTANCE)
#define I2S3_RX_DMA_IRQHandler              DMAC3_CH4_IRQHandler
#define I2S3_RX_DMA_IRQ                     DMAC3_CH4_IRQn
#define I2S3_RX_DMA_IRQ_PRIO                0
#define I2S3_RX_DMA_INSTANCE                DMA3_Channel4
#define I2S3_RX_DMA_REQUEST                 DMA_REQUEST_26
#endif
//I2S3 TX
#if defined(BSP_ENABLE_I2S3) && !defined(I2S3_TX_DMA_INSTANCE)
#define I2S3_TX_DMA_IRQHandler              DMAC3_CH5_IRQHandler
#define I2S3_TX_DMA_IRQ                     DMAC3_CH5_IRQn
#define I2S3_TX_DMA_IRQ_PRIO                0
#define I2S3_TX_DMA_INSTANCE                DMA3_Channel5
#define I2S3_TX_DMA_REQUEST                 DMA_REQUEST_27
#endif

//CODEC ADC CH0
#if defined(BSP_AUDCODEC_ADC0_DMA) && !defined(AUDCODEC_ADC0_DMA_INSTANCE)
#define AUDCODEC_ADC0_DMA_IRQHandler              DMAC3_CH4_IRQHandler
#define AUDCODEC_ADC0_DMA_IRQ                     DMAC3_CH4_IRQn
#define AUDCODEC_ADC0_DMA_IRQ_PRIO                0
#define AUDCODEC_ADC0_DMA_INSTANCE                DMA3_Channel4
#define AUDCODEC_ADC0_DMA_REQUEST                 DMA_REQUEST_28
#endif
//CODEC ADC CH1
#if defined(BSP_AUDCODEC_ADC1_DMA) && !defined(AUDCODEC_ADC1_DMA_INSTANCE)
#define AUDCODEC_ADC1_DMA_IRQHandler              DMAC3_CH5_IRQHandler
#define AUDCODEC_ADC1_DMA_IRQ                     DMAC3_CH5_IRQn
#define AUDCODEC_ADC1_DMA_IRQ_PRIO                0
#define AUDCODEC_ADC1_DMA_INSTANCE                DMA3_Channel5
#define AUDCODEC_ADC1_DMA_REQUEST                 DMA_REQUEST_29
#endif


/* DMA3 channel5 */
#if defined(BSP_UART5_TX_USING_DMA) && !defined(UART5_TX_DMA_INSTANCE)
#define UART5_DMA_TX_IRQHandler          DMAC3_CH5_IRQHandler
#define UART5_TX_DMA_IRQ_PRIO            0
#define UART5_TX_DMA_INSTANCE            DMA3_Channel5
#define UART5_TX_DMA_REQUEST             DMA_REQUEST_2
#define UART5_TX_DMA_IRQ                 DMAC3_CH5_IRQn
#endif

/* DMA3 channel5 */
#if defined(BSP_GPADC_USING_DMA) && !defined(GPADC_DMA_INSTANCE)
#define GPADC_IRQHandler              DMAC3_CH5_IRQHandler
#define GPADC_DMA_IRQ_PRIO            0
#define GPADC_DMA_INSTANCE            DMA3_Channel5
#define GPADC_DMA_REQUEST             DMA_REQUEST_30
#define GPADC_DMA_IRQ                 DMAC3_CH5_IRQn
#endif

/* DMA3 channel5 */
#if defined(BSP_SDADC_USING_DMA) && !defined(SDADC_DMA_INSTANCE)
#define SDADC_IRQHandler              DMAC3_CH5_IRQHandler
#define SDADC_DMA_IRQ_PRIO            0
#define SDADC_DMA_INSTANCE            DMA3_Channel5
#define SDADC_DMA_REQUEST             DMA_REQUEST_31
#define SDADC_DMA_IRQ                 DMAC3_CH5_IRQn
#endif

/* DMA3 channel6 */
#if defined(BSP_UART5_RX_USING_DMA) && !defined(UART5_RX_DMA_INSTANCE)
#define UART5_DMA_RX_IRQHandler          DMAC3_CH6_IRQHandler
#define UART5_RX_DMA_IRQ_PRIO            0
#define UART5_RX_DMA_INSTANCE            DMA3_Channel6
#define UART5_RX_DMA_REQUEST             DMA_REQUEST_3
#define UART5_RX_DMA_IRQ                 DMAC3_CH6_IRQn
#endif

/* DMA3 channel6, UART6 shared with UART5 RX, only one of UART5/6 is used in most cases. Comment this out if want to use DMA3_CH8*/
#if defined(BSP_UART6_RX_USING_DMA) && !defined(UART6_RX_DMA_INSTANCE)
#define UART6_DMA_RX_IRQHandler          DMAC3_CH6_IRQHandler
#define UART6_RX_DMA_IRQ_PRIO            0
#define UART6_RX_DMA_INSTANCE            DMA3_Channel6
#define UART6_RX_DMA_REQUEST             DMA_REQUEST_5
#define UART6_RX_DMA_IRQ                 DMAC3_CH6_IRQn
#endif

/* DMA3 channel7 */
#if defined(BSP_SPI4_RX_USING_DMA) && !defined(SPI4_RX_DMA_INSTANCE)
#define SPI4_DMA_RX_IRQHandler         DMAC3_CH7_IRQHandler
#define SPI4_RX_DMA_IRQ_PRIO           0
#define SPI4_RX_DMA_INSTANCE           DMA3_Channel7
#define SPI4_RX_DMA_REQUEST            DMA_REQUEST_19
#define SPI4_RX_DMA_IRQ                DMAC3_CH7_IRQn
#endif

/* DMA3 channel8 */
#if defined(BSP_I2C7_USING_DMA) && !defined(I2C7_DMA_INSTANCE)
#define I2C7_DMA_IRQHandler              DMAC3_CH8_IRQHandler
#define I2C7_DMA_IRQ_PRIO                1
#define I2C7_DMA_INSTANCE                DMA3_Channel8
#define I2C7_DMA_REQUEST                 DMA_REQUEST_23
#define I2C7_DMA_IRQ                     DMAC3_CH8_IRQn
#endif

/* DMA3 channel8 */
#if defined(BSP_SPI4_TX_USING_DMA) && !defined(SPI4_TX_DMA_INSTANCE)
#define SPI4_DMA_TX_IRQHandler         DMAC3_CH8_IRQHandler
#define SPI4_TX_DMA_IRQ_PRIO           0
#define SPI4_TX_DMA_INSTANCE           DMA3_Channel8
#define SPI4_TX_DMA_REQUEST            DMA_REQUEST_18
#define SPI4_TX_DMA_IRQ                DMAC3_CH8_IRQn
#endif

/* DMA3 channel7 */
#if defined(BSP_UART6_TX_USING_DMA) && !defined(UART6_TX_DMA_INSTANCE)
#define UART6_DMA_TX_IRQHandler          DMAC3_CH7_IRQHandler
#define UART6_TX_DMA_IRQ_PRIO            0
#define UART6_TX_DMA_INSTANCE            DMA3_Channel7
#define UART6_TX_DMA_REQUEST             DMA_REQUEST_4
#define UART6_TX_DMA_IRQ                 DMAC3_CH7_IRQn
#endif


/* DMA3 channel8 */
#if defined(BSP_UART6_RX_USING_DMA) && !defined(UART6_RX_DMA_INSTANCE)
#define UART6_DMA_RX_IRQHandler          DMAC3_CH8_IRQHandler
#define UART6_RX_DMA_IRQ_PRIO            0
#define UART6_RX_DMA_INSTANCE            DMA3_Channel8
#define UART6_RX_DMA_REQUEST             DMA_REQUEST_5
#define UART6_RX_DMA_IRQ                 DMAC3_CH8_IRQn
#endif

/* DMA3 channel8 */
#ifndef FLASH5_DMA_INSTANCE
#define FLASH5_IRQHandler              DMAC3_CH8_IRQHandler
#define FLASH5_DMA_IRQ_PRIO            0
#define FLASH5_DMA_INSTANCE            DMA3_Channel8
#define FLASH5_DMA_REQUEST             DMA_REQUEST_20
#define FLASH5_DMA_IRQ                 DMAC3_CH8_IRQn
#endif

#ifdef __cplusplus
}
#endif

#endif /* __DMA_CONFIG_H__ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
