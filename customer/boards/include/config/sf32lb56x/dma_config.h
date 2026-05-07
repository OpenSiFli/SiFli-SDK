/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DMA_CONFIG_H__
#define __DMA_CONFIG_H__

#include <rtconfig.h>

#ifdef __cplusplus
extern "C" {
#endif

/*************************************DMA1 ***************************************/
#define FLASH1_DMA_REQUEST                     DMA_REQUEST_0
#define FLASH2_DMA_REQUEST                     DMA_REQUEST_1
#define FLASH3_DMA_REQUEST                     DMA_REQUEST_2
#define I2C4_DMA_REQUEST                       DMA_REQUEST_3
#define UART1_TX_DMA_REQUEST                   DMA_REQUEST_4
#define UART1_RX_DMA_REQUEST                   DMA_REQUEST_5
#define UART2_TX_DMA_REQUEST                   DMA_REQUEST_6
#define UART2_RX_DMA_REQUEST                   DMA_REQUEST_7
#define GPTIM1_UPDATE_DMA_REQUEST              DMA_REQUEST_8
#define GPTIM1_TRIGGER_DMA_REQUEST             DMA_REQUEST_9
#define GPTIM1_CC1_DMA_REQUEST                 DMA_REQUEST_10
#define GPTIM1_CC2_DMA_REQUEST                 DMA_REQUEST_11
#define GPTIM1_CC3_DMA_REQUEST                 DMA_REQUEST_12
#define GPTIM1_CC4_DMA_REQUEST                 DMA_REQUEST_13
#define PWM2_UPDATE_DMA_REQUEST                GPTIM1_UPDATE_DMA_REQUEST
#define PWM2_TRIGGER_DMA_REQUEST               GPTIM1_TRIGGER_DMA_REQUEST
#define PWM2_CC1_DMA_REQUEST                   GPTIM1_CC1_DMA_REQUEST
#define PWM2_CC2_DMA_REQUEST                   GPTIM1_CC2_DMA_REQUEST
#define PWM2_CC3_DMA_REQUEST                   GPTIM1_CC3_DMA_REQUEST
#define PWM2_CC4_DMA_REQUEST                   GPTIM1_CC4_DMA_REQUEST
#define BTIM1_DMA_REQUEST                      DMA_REQUEST_14
#define BTIM2_DMA_REQUEST                      DMA_REQUEST_15
#define ATIM1_UPDATE_DMA_REQUEST               DMA_REQUEST_16
#define ATIM1_TRIGGER_DMA_REQUEST              DMA_REQUEST_17
#define ATIM1_CC1_DMA_REQUEST                  DMA_REQUEST_18
#define ATIM1_CC2_DMA_REQUEST                  DMA_REQUEST_19
#define ATIM1_CC3_DMA_REQUEST                  DMA_REQUEST_20
#define ATIM1_CC4_DMA_REQUEST                  DMA_REQUEST_21
#define PWMA1_UPDATE_DMA_REQUEST               ATIM1_UPDATE_DMA_REQUEST
#define PWMA1_TRIGGER_DMA_REQUEST              ATIM1_TRIGGER_DMA_REQUEST
#define PWMA1_CC1_DMA_REQUEST                  ATIM1_CC1_DMA_REQUEST
#define PWMA1_CC2_DMA_REQUEST                  ATIM1_CC2_DMA_REQUEST
#define PWMA1_CC3_DMA_REQUEST                  ATIM1_CC3_DMA_REQUEST
#define PWMA1_CC4_DMA_REQUEST                  ATIM1_CC4_DMA_REQUEST
#define I2C1_DMA_REQUEST                       DMA_REQUEST_22
#define I2C2_DMA_REQUEST                       DMA_REQUEST_23
#define I2C3_DMA_REQUEST                       DMA_REQUEST_24
#define ATIM1_COM_DMA_REQUEST                  DMA_REQUEST_25
#define UART3_TX_DMA_REQUEST                   DMA_REQUEST_26
#define UART3_RX_DMA_REQUEST                   DMA_REQUEST_27
#define SPI1_TX_DMA_REQUEST                    DMA_REQUEST_28
#define SPI1_RX_DMA_REQUEST                    DMA_REQUEST_29
#define SPI2_TX_DMA_REQUEST                    DMA_REQUEST_30
#define SPI2_RX_DMA_REQUEST                    DMA_REQUEST_31
#define I2S_TX_DMA_REQUEST                     DMA_REQUEST_32
#define I2S_RX_DMA_REQUEST                     DMA_REQUEST_33
#define SCI_TX_DMA_REQUEST                     DMA_REQUEST_34
#define SCI_RX_DMA_REQUEST                     DMA_REQUEST_35
#define PDM1_L_DMA_REQUEST                     DMA_REQUEST_36
#define PDM1_R_DMA_REQUEST                     DMA_REQUEST_37
#define PDM2_L_DMA_REQUEST                     DMA_REQUEST_38
#define PDM2_R_DMA_REQUEST                     DMA_REQUEST_39
#define AUDCODEC_DAC0_DMA_REQUEST              DMA_REQUEST_41
#define AUDCODEC_DAC1_DMA_REQUEST              DMA_REQUEST_42
#define GPTIM2_UPDATE_DMA_REQUEST              DMA_REQUEST_43
#define GPTIM2_TRIGGER_DMA_REQUEST             DMA_REQUEST_44
#define GPTIM2_CC1_DMA_REQUEST                 DMA_REQUEST_45
#define PWM3_UPDATE_DMA_REQUEST                GPTIM2_UPDATE_DMA_REQUEST
#define PWM3_TRIGGER_DMA_REQUEST               GPTIM2_TRIGGER_DMA_REQUEST
#define PWM3_CC1_DMA_REQUEST                   GPTIM2_CC1_DMA_REQUEST
#define AUDPRC_TX_OUT1_DMA_REQUEST             DMA_REQUEST_46
#define AUDPRC_TX_OUT0_DMA_REQUEST             DMA_REQUEST_47
#define AUDPRC_TX3_DMA_REQUEST                 DMA_REQUEST_48
#define AUDPRC_TX2_DMA_REQUEST                 DMA_REQUEST_49
#define AUDPRC_TX1_DMA_REQUEST                 DMA_REQUEST_50
#define AUDPRC_TX0_DMA_REQUEST                 DMA_REQUEST_51
#define AUDPRC_RX1_DMA_REQUEST                 DMA_REQUEST_52
#define AUDPRC_RX0_DMA_REQUEST                 DMA_REQUEST_53
#define GPTIM2_CC2_DMA_REQUEST                 DMA_REQUEST_54
#define GPTIM2_CC3_DMA_REQUEST                 DMA_REQUEST_55
#define GPTIM2_CC4_DMA_REQUEST                 DMA_REQUEST_56
#define PWM3_CC2_DMA_REQUEST                   GPTIM2_CC2_DMA_REQUEST
#define PWM3_CC3_DMA_REQUEST                   GPTIM2_CC3_DMA_REQUEST
#define PWM3_CC4_DMA_REQUEST                   GPTIM2_CC4_DMA_REQUEST
#define SDMMC2_DMA_REQUEST                     DMA_REQUEST_57

/* DMA1 channel1 */
#define FLASH1_IRQHandler              DMAC1_CH1_IRQHandler
#define FLASH1_DMA_IRQ_PRIO            0
#define FLASH1_DMA_INSTANCE            DMA1_Channel1
#define FLASH1_DMA_IRQ                 DMAC1_CH1_IRQn

#define AUDPRC_TX0_DMA_IRQHandler              DMAC1_CH1_IRQHandler
#define AUDPRC_TX0_DMA_IRQ_PRIO                0
#define AUDPRC_TX0_DMA_INSTANCE                DMA1_Channel1

#define AUDPRC_TX0_DMA_IRQ                     DMAC1_CH1_IRQn

#define AUDPRC_RX1_DMA_IRQHandler              DMAC1_CH1_IRQHandler
#define AUDPRC_RX1_DMA_IRQ_PRIO                0
#define AUDPRC_RX1_DMA_INSTANCE                DMA1_Channel1
#define AUDPRC_RX1_DMA_IRQ                     DMAC1_CH1_IRQn

/* DMA1 channel2 */
#define FLASH2_IRQHandler              DMAC1_CH2_IRQHandler
#define FLASH2_DMA_IRQ_PRIO            0
#define FLASH2_DMA_INSTANCE            DMA1_Channel2
#define FLASH2_DMA_IRQ                 DMAC1_CH2_IRQn

#define AUDPRC_TX1_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define AUDPRC_TX1_DMA_IRQ_PRIO                0
#define AUDPRC_TX1_DMA_INSTANCE                DMA1_Channel2
#define AUDPRC_TX1_DMA_IRQ                     DMAC1_CH2_IRQn

#define AUDPRC_TX_OUT0_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define AUDPRC_TX_OUT0_DMA_IRQ_PRIO                0
#define AUDPRC_TX_OUT0_DMA_INSTANCE                DMA1_Channel2
#define AUDPRC_TX_OUT0_DMA_IRQ                     DMAC1_CH2_IRQn

// AUDPRC RX CH0
#define AUDPRC_RX0_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define AUDPRC_RX0_DMA_IRQ_PRIO                0
#define AUDPRC_RX0_DMA_INSTANCE                DMA1_Channel2
#define AUDPRC_RX0_DMA_IRQ                     DMAC1_CH2_IRQn

//GTIM1 PWM2
#if defined(BSP_PWM2_UPDATE_USING_DMA) && !defined(PWM2_UPDATE_DMA_INSTANCE)
#define PWM2_UPDATE_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define PWM2_UPDATE_DMA_IRQ_PRIO                1
#define PWM2_UPDATE_DMA_INSTANCE                DMA1_Channel2
#define PWM2_UPDATE_DMA_IRQ                     DMAC1_CH2_IRQn
#define PWM2_UPDATE_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM2_UPDATE_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWM2_CC1_USING_DMA) && !defined(PWM2_CC1_DMA_INSTANCE)
#define PWM2_CC1_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define PWM2_CC1_DMA_IRQ_PRIO                1
#define PWM2_CC1_DMA_INSTANCE                DMA1_Channel2
#define PWM2_CC1_DMA_IRQ                     DMAC1_CH2_IRQn
#define PWM2_CC1_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM2_CC1_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWM2_CC2_USING_DMA) && !defined(PWM2_CC2_DMA_INSTANCE)
#define PWM2_CC2_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define PWM2_CC2_DMA_IRQ_PRIO                1
#define PWM2_CC2_DMA_INSTANCE                DMA1_Channel2
#define PWM2_CC2_DMA_IRQ                     DMAC1_CH2_IRQn
#define PWM2_CC2_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM2_CC2_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWM2_CC3_USING_DMA) && !defined(PWM2_CC3_DMA_INSTANCE)
#define PWM2_CC3_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define PWM2_CC3_DMA_IRQ_PRIO                1
#define PWM2_CC3_DMA_INSTANCE                DMA1_Channel2
#define PWM2_CC3_DMA_IRQ                     DMAC1_CH2_IRQn
#define PWM2_CC3_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM2_CC3_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWM2_CC4_USING_DMA) && !defined(PWM2_CC4_DMA_INSTANCE)
#define PWM2_CC4_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define PWM2_CC4_DMA_IRQ_PRIO                1
#define PWM2_CC4_DMA_INSTANCE                DMA1_Channel2
#define PWM2_CC4_DMA_IRQ                     DMAC1_CH2_IRQn
#define PWM2_CC4_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM2_CC4_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

//GTIM2 PWM3
#if defined(BSP_PWM3_UPDATE_USING_DMA) && !defined(PWM3_UPDATE_DMA_INSTANCE)
#define PWM3_UPDATE_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define PWM3_UPDATE_DMA_IRQ_PRIO                1
#define PWM3_UPDATE_DMA_INSTANCE                DMA1_Channel2
#define PWM3_UPDATE_DMA_IRQ                     DMAC1_CH2_IRQn
#define PWM3_UPDATE_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM3_UPDATE_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWM3_CC1_USING_DMA) && !defined(PWM3_CC1_DMA_INSTANCE)
#define PWM3_CC1_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define PWM3_CC1_DMA_IRQ_PRIO                1
#define PWM3_CC1_DMA_INSTANCE                DMA1_Channel2
#define PWM3_CC1_DMA_IRQ                     DMAC1_CH2_IRQn
#define PWM3_CC1_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM3_CC1_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWM3_CC2_USING_DMA) && !defined(PWM3_CC2_DMA_INSTANCE)
#define PWM3_CC2_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define PWM3_CC2_DMA_IRQ_PRIO                1
#define PWM3_CC2_DMA_INSTANCE                DMA1_Channel2
#define PWM3_CC2_DMA_IRQ                     DMAC1_CH2_IRQn
#define PWM3_CC2_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM3_CC2_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWM3_CC3_USING_DMA) && !defined(PWM3_CC3_DMA_INSTANCE)
#define PWM3_CC3_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define PWM3_CC3_DMA_IRQ_PRIO                1
#define PWM3_CC3_DMA_INSTANCE                DMA1_Channel2
#define PWM3_CC3_DMA_IRQ                     DMAC1_CH2_IRQn
#define PWM3_CC3_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM3_CC3_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWM3_CC4_USING_DMA) && !defined(PWM2_CC3_DMA_INSTANCE)
#define PWM3_CC4_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define PWM3_CC4_DMA_IRQ_PRIO                1
#define PWM3_CC4_DMA_INSTANCE                DMA1_Channel2
#define PWM3_CC4_DMA_IRQ                     DMAC1_CH2_IRQn
#define PWM3_CC4_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM3_CC4_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

//ATIM1 PWMA1
#if defined(BSP_PWMA1_UPDATE_USING_DMA) && !defined(PWMA1_UPDATE_DMA_INSTANCE)
#define PWMA1_UPDATE_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define PWMA1_UPDATE_DMA_IRQ_PRIO                1
#define PWMA1_UPDATE_DMA_INSTANCE                DMA1_Channel2
#define PWMA1_UPDATE_DMA_IRQ                     DMAC1_CH2_IRQn
#define PWMA1_UPDATE_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWMA1_UPDATE_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWMA1_CC1_USING_DMA) && !defined(PWMA1_CC1_DMA_INSTANCE)
#define PWMA1_CC1_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define PWMA1_CC1_DMA_IRQ_PRIO                1
#define PWMA1_CC1_DMA_INSTANCE                DMA1_Channel2
#define PWMA1_CC1_DMA_IRQ                     DMAC1_CH2_IRQn
#define PWMA1_CC1_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWMA1_CC1_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWMA1_CC2_USING_DMA) && !defined(PWMA1_CC2_DMA_INSTANCE)
#define PWMA1_CC2_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define PWMA1_CC2_DMA_IRQ_PRIO                1
#define PWMA1_CC2_DMA_INSTANCE                DMA1_Channel2
#define PWMA1_CC2_DMA_IRQ                     DMAC1_CH2_IRQn
#define PWMA1_CC2_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWMA1_CC2_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWMA1_CC3_USING_DMA) && !defined(PWMA1_CC3_DMA_INSTANCE)
#define PWMA1_CC3_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define PWMA1_CC3_DMA_IRQ_PRIO                1
#define PWMA1_CC3_DMA_INSTANCE                DMA1_Channel2
#define PWMA1_CC3_DMA_IRQ                     DMAC1_CH2_IRQn
#define PWMA1_CC3_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWMA1_CC3_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWMA1_CC4_USING_DMA) && !defined(PWMA1_CC4_DMA_INSTANCE)
#define PWMA1_CC4_DMA_IRQHandler              DMAC1_CH2_IRQHandler
#define PWMA1_CC4_DMA_IRQ_PRIO                1
#define PWMA1_CC4_DMA_INSTANCE                DMA1_Channel2
#define PWMA1_CC4_DMA_IRQ                     DMAC1_CH2_IRQn
#define PWMA1_CC4_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWMA1_CC4_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

/* DMA1 channel3 */
#define FLASH3_IRQHandler              DMAC1_CH3_IRQHandler
#define FLASH3_DMA_IRQ_PRIO            0
#define FLASH3_DMA_INSTANCE            DMA1_Channel3
#define FLASH3_DMA_IRQ                 DMAC1_CH3_IRQn

#define SDMMC2_DMA_IRQHandler          DMAC1_CH3_IRQHandler
#define SDMMC2_DMA_IRQ_PRIO            0
#define SDMMC2_DMA_INSTANCE            DMA1_Channel3
#define SDMMC2_DMA_IRQ                 DMAC1_CH3_IRQn

#define SPI1_DMA_RX_IRQHandler       DMAC1_CH3_IRQHandler
#define SPI1_RX_DMA_IRQ_PRIO         0
#define SPI1_RX_DMA_INSTANCE         DMA1_Channel3
#define SPI1_RX_DMA_IRQ              DMAC1_CH3_IRQn

/* DMA1 channel4  */
//PDM1 L
#define PDM1_L_DMA_IRQHandler           DMAC1_CH4_IRQHandler
#define PDM1_L_DMA_IRQ_PRIO                  0
#define PDM1_L_DMA_INSTANCE             DMA1_Channel4
#define PDM1_L_DMA_IRQ                  DMAC1_CH4_IRQn

#if !defined(PDM2_L_DMA_INSTANCE) && defined(BSP_USING_PDM2)
#define PDM2_L_DMA_IRQHandler           DMAC1_CH3_IRQHandler
#define PDM2_L_DMA_IRQ_PRIO             0
#define PDM2_L_DMA_INSTANCE             DMA1_Channel3
#define PDM2_L_DMA_IRQ                  DMAC1_CH3_IRQn
#endif

#define I2S_TX_DMA_IRQHandler              DMAC1_CH4_IRQHandler
#define I2S_TX_DMA_IRQ_PRIO                     0
#define I2S_TX_DMA_INSTANCE                DMA1_Channel4
#define I2S_TX_DMA_IRQ                     DMAC1_CH4_IRQn

#define SPI1_DMA_TX_IRQHandler       DMAC1_CH4_IRQHandler
#define SPI1_TX_DMA_IRQ_PRIO         0
#define SPI1_TX_DMA_INSTANCE         DMA1_Channel4
#define SPI1_TX_DMA_IRQ              DMAC1_CH4_IRQn

/* DMA1 channel5 */
#define UART1_DMA_TX_IRQHandler          DMAC1_CH5_IRQHandler
#define UART1_TX_DMA_IRQ_PRIO            0
#define UART1_TX_DMA_INSTANCE            DMA1_Channel5
#define UART1_TX_DMA_IRQ                 DMAC1_CH5_IRQn

/*UART 2 RX DMA, shared with UART 1 TX DMA*/
#define UART2_DMA_RX_IRQHandler          DMAC1_CH5_IRQHandler
#define UART2_RX_DMA_IRQ_PRIO            0
#define UART2_RX_DMA_INSTANCE            DMA1_Channel5
#define UART2_RX_DMA_IRQ                 DMAC1_CH5_IRQn

#define AUDPRC_TX3_DMA_IRQHandler              DMAC1_CH5_IRQHandler
#define AUDPRC_TX3_DMA_IRQ_PRIO                0
#define AUDPRC_TX3_DMA_INSTANCE                DMA1_Channel5
#define AUDPRC_TX3_DMA_IRQ                     DMAC1_CH5_IRQn

/* DMA1 channel6 */
#define UART1_DMA_RX_IRQHandler          DMAC1_CH6_IRQHandler
#define UART1_RX_DMA_IRQ_PRIO            0
#define UART1_RX_DMA_INSTANCE            DMA1_Channel6
#define UART1_RX_DMA_IRQ                 DMAC1_CH6_IRQn

#define UART2_DMA_TX_IRQHandler          DMAC1_CH6_IRQHandler
#define UART2_TX_DMA_IRQ_PRIO            0
#define UART2_TX_DMA_INSTANCE            DMA1_Channel6
#define UART2_TX_DMA_IRQ                 DMAC1_CH6_IRQn

/* DMA1 channel7  */
#define UART3_DMA_TX_IRQHandler         DMAC1_CH7_IRQHandler
#define UART3_TX_DMA_IRQ_PRIO           0
#define UART3_TX_DMA_INSTANCE           DMA1_Channel7
#define UART3_TX_DMA_IRQ                DMAC1_CH7_IRQn

/* DMA1 channel7  */
//PDM1 R
#define PDM1_R_DMA_IRQHandler              DMAC1_CH7_IRQHandler
#define PDM1_R_DMA_IRQ_PRIO                     0
#define PDM1_R_DMA_INSTANCE                DMA1_Channel7
#define PDM1_R_DMA_IRQ                     DMAC1_CH7_IRQn

#define I2S_RX_DMA_IRQHandler              DMAC1_CH7_IRQHandler
#define I2S_RX_DMA_IRQ_PRIO                     0
#define I2S_RX_DMA_INSTANCE                DMA1_Channel7
#define I2S_RX_DMA_IRQ                     DMAC1_CH7_IRQn

#define SPI2_DMA_RX_IRQHandler         DMAC1_CH7_IRQHandler
#define SPI2_RX_DMA_IRQ_PRIO           0
#define SPI2_RX_DMA_INSTANCE           DMA1_Channel7
#define SPI2_RX_DMA_IRQ                DMAC1_CH7_IRQn

/* DMA1 channel8  */
#define UART3_DMA_RX_IRQHandler         DMAC1_CH8_IRQHandler
#define UART3_RX_DMA_IRQ_PRIO           0
#define UART3_RX_DMA_INSTANCE           DMA1_Channel8
#define UART3_RX_DMA_IRQ                DMAC1_CH8_IRQn

#define AUDPRC_TX2_DMA_IRQHandler              DMAC1_CH8_IRQHandler
#define AUDPRC_TX2_DMA_IRQ_PRIO                0
#define AUDPRC_TX2_DMA_INSTANCE                DMA1_Channel8
#define AUDPRC_TX2_DMA_IRQ                     DMAC1_CH8_IRQn

#define SPI2_DMA_TX_IRQHandler         DMAC1_CH8_IRQHandler
#define SPI2_TX_DMA_IRQ_PRIO           0
#define SPI2_TX_DMA_INSTANCE           DMA1_Channel8
#define SPI2_TX_DMA_IRQ                DMAC1_CH8_IRQn

/*************************************DMA2 ***************************************/
#define UART4_TX_DMA_REQUEST                   DMA_REQUEST_0
#define UART4_RX_DMA_REQUEST                   DMA_REQUEST_1
#define UART5_TX_DMA_REQUEST                   DMA_REQUEST_2
#define UART5_RX_DMA_REQUEST                   DMA_REQUEST_3
#define UART6_TX_DMA_REQUEST                   DMA_REQUEST_4
#define UART6_RX_DMA_REQUEST                   DMA_REQUEST_5
#define BTIM3_DMA_REQUEST                      DMA_REQUEST_6
#define BTIM4_DMA_REQUEST                      DMA_REQUEST_7
#define GPTIM3_UPDATE_DMA_REQUEST              DMA_REQUEST_8
#define GPTIM3_TRIGGER_DMA_REQUEST             DMA_REQUEST_9
#define GPTIM3_CC1_DMA_REQUEST                 DMA_REQUEST_10
#define GPTIM3_CC2_DMA_REQUEST                 DMA_REQUEST_11
#define GPTIM3_CC3_DMA_REQUEST                 DMA_REQUEST_12
#define GPTIM3_CC4_DMA_REQUEST                 DMA_REQUEST_13
#define PWM4_UPDATE_DMA_REQUEST                GPTIM3_UPDATE_DMA_REQUEST
#define PWM4_TRIGGER_DMA_REQUEST               GPTIM3_TRIGGER_DMA_REQUEST
#define PWM4_CC1_DMA_REQUEST                   GPTIM3_CC1_DMA_REQUEST
#define PWM4_CC2_DMA_REQUEST                   GPTIM3_CC2_DMA_REQUEST
#define PWM4_CC3_DMA_REQUEST                   GPTIM3_CC3_DMA_REQUEST
#define PWM4_CC4_DMA_REQUEST                   GPTIM3_CC4_DMA_REQUEST
#define GPTIM5_UPDATE_DMA_REQUEST              DMA_REQUEST_14
#define GPTIM5_TRIGGER_DMA_REQUEST             DMA_REQUEST_15
#define PWM6_UPDATE_DMA_REQUEST                GPTIM5_UPDATE_DMA_REQUEST
#define PWM6_TRIGGER_DMA_REQUEST               GPTIM5_TRIGGER_DMA_REQUEST
#define SPI3_TX_DMA_REQUEST                    DMA_REQUEST_16
#define SPI3_RX_DMA_REQUEST                    DMA_REQUEST_17
#define SPI4_TX_DMA_REQUEST                    DMA_REQUEST_18
#define SPI4_RX_DMA_REQUEST                    DMA_REQUEST_19
#define FLASH5_DMA_REQUEST                     DMA_REQUEST_20
#define I2C5_DMA_REQUEST                       DMA_REQUEST_21
#define I2C6_DMA_REQUEST                       DMA_REQUEST_22
#define I2C7_DMA_REQUEST                       DMA_REQUEST_23
#define GPTIM4_UPDATE_DMA_REQUEST              DMA_REQUEST_24
#define GPTIM4_TRIGGER_DMA_REQUEST             DMA_REQUEST_25
#define GPTIM4_CC1_DMA_REQUEST                 DMA_REQUEST_26
#define GPTIM4_CC2_DMA_REQUEST                 DMA_REQUEST_27
#define GPTIM4_CC3_DMA_REQUEST                 DMA_REQUEST_28
#define GPTIM4_CC4_DMA_REQUEST                 DMA_REQUEST_29
#define PWM5_UPDATE_DMA_REQUEST                GPTIM4_UPDATE_DMA_REQUEST
#define PWM5_TRIGGER_DMA_REQUEST               GPTIM4_TRIGGER_DMA_REQUEST
#define PWM5_CC1_DMA_REQUEST                   GPTIM4_CC1_DMA_REQUEST
#define PWM5_CC2_DMA_REQUEST                   GPTIM4_CC2_DMA_REQUEST
#define PWM5_CC3_DMA_REQUEST                   GPTIM4_CC3_DMA_REQUEST
#define PWM5_CC4_DMA_REQUEST                   GPTIM4_CC4_DMA_REQUEST

#define GPADC_DMA_REQUEST                      DMA_REQUEST_30

/* DMA2 channel1  */
#if defined(BSP_UART4_TX_USING_DMA) && !defined(UART4_TX_DMA_INSTANCE)
#define UART4_DMA_TX_IRQHandler         DMAC2_CH1_IRQHandler
#define UART4_TX_DMA_IRQ_PRIO           0
#define UART4_TX_DMA_INSTANCE           DMA2_Channel1
#define UART4_TX_DMA_IRQ                DMAC2_CH1_IRQn
#endif
/* DMA2 channel1  */

//GTIM3 PWM4
#if defined(BSP_PWM4_UPDATE_USING_DMA) && !defined(PWM4_UPDATE_DMA_INSTANCE)
#define PWM4_UPDATE_DMA_IRQHandler              DMAC2_CH1_IRQHandler
#define PWM4_UPDATE_DMA_IRQ_PRIO                1
#define PWM4_UPDATE_DMA_INSTANCE                DMA2_Channel1
#define PWM4_UPDATE_DMA_IRQ                     DMAC2_CH1_IRQn
#define PWM4_UPDATE_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM4_UPDATE_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWM4_CC1_USING_DMA) && !defined(PWM4_CC1_DMA_INSTANCE)
#define PWM4_CC1_DMA_IRQHandler              DMAC2_CH1_IRQHandler
#define PWM4_CC1_DMA_IRQ_PRIO                1
#define PWM4_CC1_DMA_INSTANCE                DMA2_Channel1
#define PWM4_CC1_DMA_IRQ                     DMAC2_CH1_IRQn
#define PWM4_CC1_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM4_CC1_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWM4_CC2_USING_DMA) && !defined(PWM4_CC2_DMA_INSTANCE)
#define PWM4_CC2_DMA_IRQHandler              DMAC2_CH1_IRQHandler
#define PWM4_CC2_DMA_IRQ_PRIO                1
#define PWM4_CC2_DMA_INSTANCE                DMA2_Channel1
#define PWM4_CC2_DMA_IRQ                     DMAC2_CH1_IRQn
#define PWM4_CC2_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM4_CC2_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWM4_CC3_USING_DMA) && !defined(PWM4_CC3_DMA_INSTANCE)
#define PWM4_CC3_DMA_IRQHandler              DMAC2_CH1_IRQHandler
#define PWM4_CC3_DMA_IRQ_PRIO                1
#define PWM4_CC3_DMA_INSTANCE                DMA2_Channel1
#define PWM4_CC3_DMA_IRQ                     DMAC2_CH1_IRQn
#define PWM4_CC3_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM4_CC3_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWM4_CC4_USING_DMA) && !defined(PWM4_CC4_DMA_INSTANCE)
#define PWM4_CC4_DMA_IRQHandler              DMAC2_CH1_IRQHandler
#define PWM4_CC4_DMA_IRQ_PRIO                1
#define PWM4_CC4_DMA_INSTANCE                DMA2_Channel1
#define PWM4_CC4_DMA_IRQ                     DMAC2_CH1_IRQn
#define PWM4_CC4_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM4_CC4_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

//GTIM4 PW5
#if defined(BSP_PWM5_UPDATE_USING_DMA) && !defined(PWM5_UPDATE_DMA_INSTANCE)
#define PWM5_UPDATE_DMA_IRQHandler              DMAC2_CH1_IRQHandler
#define PWM5_UPDATE_DMA_IRQ_PRIO                1
#define PWM5_UPDATE_DMA_INSTANCE                DMA2_Channel1
#define PWM5_UPDATE_DMA_IRQ                     DMAC2_CH1_IRQn
#define PWM5_UPDATE_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM5_UPDATE_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWM5_CC1_USING_DMA) && !defined(PWM5_CC1_DMA_INSTANCE)
#define PWM5_CC1_DMA_IRQHandler              DMAC2_CH1_IRQHandler
#define PWM5_CC1_DMA_IRQ_PRIO                1
#define PWM5_CC1_DMA_INSTANCE                DMA2_Channel1
#define PWM5_CC1_DMA_IRQ                     DMAC2_CH1_IRQn
#define PWM5_CC1_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM5_CC1_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWM5_CC2_USING_DMA) && !defined(PWM5_CC2_DMA_INSTANCE)
#define PWM5_CC2_DMA_IRQHandler              DMAC2_CH1_IRQHandler
#define PWM5_CC2_DMA_IRQ_PRIO                1
#define PWM5_CC2_DMA_INSTANCE                DMA2_Channel1
#define PWM5_CC2_DMA_IRQ                     DMAC2_CH1_IRQn
#define PWM5_CC2_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM5_CC2_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWM5_CC3_USING_DMA) && !defined(PWM5_CC3_DMA_INSTANCE)
#define PWM5_CC3_DMA_IRQHandler              DMAC2_CH1_IRQHandler
#define PWM5_CC3_DMA_IRQ_PRIO                1
#define PWM5_CC3_DMA_INSTANCE                DMA2_Channel1
#define PWM5_CC3_DMA_IRQ                     DMAC2_CH1_IRQn
#define PWM5_CC3_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM5_CC3_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

#if defined(BSP_PWM5_CC4_USING_DMA) && !defined(PWM5_CC4_DMA_INSTANCE)
#define PWM5_CC4_DMA_IRQHandler              DMAC2_CH1_IRQHandler
#define PWM5_CC4_DMA_IRQ_PRIO                1
#define PWM5_CC4_DMA_INSTANCE                DMA2_Channel1
#define PWM5_CC4_DMA_IRQ                     DMAC2_CH1_IRQn
#define PWM5_CC4_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM5_CC4_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

//GTIM5 PW6
#if defined(BSP_PWM6_UPDATE_USING_DMA) && !defined(PWM6_UPDATE_DMA_INSTANCE)
#define PWM6_UPDATE_DMA_IRQHandler              DMAC2_CH1_IRQHandler
#define PWM6_UPDATE_DMA_IRQ_PRIO                1
#define PWM6_UPDATE_DMA_INSTANCE                DMA2_Channel1
#define PWM6_UPDATE_DMA_IRQ                     DMAC2_CH1_IRQn
#define PWM6_UPDATE_DMA_PDATAALIGN              DMA_PDATAALIGN_HALFWORD
#define PWM6_UPDATE_DMA_MDATAALIGN              DMA_MDATAALIGN_HALFWORD
#endif

/* DMA2 channel2  */
#if defined(BSP_UART4_RX_USING_DMA) && !defined(UART4_RX_DMA_INSTANCE)
#define UART4_DMA_RX_IRQHandler         DMAC2_CH2_IRQHandler
#define UART4_RX_DMA_IRQ_PRIO           0
#define UART4_RX_DMA_INSTANCE           DMA2_Channel2
#define UART4_RX_DMA_IRQ                DMAC2_CH2_IRQn
#endif

/* DMA2 channel3  */
#define SPI3_DMA_RX_IRQHandler         DMAC2_CH3_IRQHandler
#define SPI3_RX_DMA_IRQ_PRIO           0
#define SPI3_RX_DMA_INSTANCE           DMA2_Channel3
#define SPI3_RX_DMA_IRQ                DMAC2_CH3_IRQn

/* DMA2 channel4  */
#define SPI3_DMA_TX_IRQHandler         DMAC2_CH4_IRQHandler
#define SPI3_TX_DMA_IRQ_PRIO           0
#define SPI3_TX_DMA_INSTANCE           DMA2_Channel4
#define SPI3_TX_DMA_IRQ                DMAC2_CH4_IRQn

/* DMA2 channel3 */
#define I2C5_DMA_IRQHandler              DMAC2_CH3_IRQHandler
#define I2C5_DMA_IRQ_PRIO                1
#define I2C5_DMA_INSTANCE                DMA2_Channel3
#define I2C5_DMA_IRQ                     DMAC2_CH3_IRQn

/* DMA2 channel4 */
#define I2C6_DMA_IRQHandler              DMAC2_CH4_IRQHandler
#define I2C6_DMA_IRQ_PRIO                1
#define I2C6_DMA_INSTANCE                DMA2_Channel4
#define I2C6_DMA_IRQ                     DMAC2_CH4_IRQn

/* DMA2 channel5 */
#define UART5_DMA_TX_IRQHandler          DMAC2_CH5_IRQHandler
#define UART5_TX_DMA_IRQ_PRIO            0
#define UART5_TX_DMA_INSTANCE            DMA2_Channel5
#define UART5_TX_DMA_IRQ                 DMAC2_CH5_IRQn

/* DMA2 channel5 */
#define GPADC_DMA_IRQHandler              DMAC2_CH5_IRQHandler
#define GPADC_DMA_IRQ_PRIO                 0
#define GPADC_DMA_INSTANCE            DMA2_Channel5
#define GPADC_DMA_IRQ                 DMAC2_CH5_IRQn

/* DMA2 channel6 */
#define UART5_DMA_RX_IRQHandler          DMAC2_CH6_IRQHandler
#define UART5_RX_DMA_IRQ_PRIO            0
#define UART5_RX_DMA_INSTANCE            DMA2_Channel6
#define UART5_RX_DMA_IRQ                 DMAC2_CH6_IRQn

/* DMA2 channel7 */
#define SPI4_DMA_RX_IRQHandler         DMAC2_CH7_IRQHandler
#define SPI4_RX_DMA_IRQ_PRIO           0
#define SPI4_RX_DMA_INSTANCE           DMA2_Channel7
#define SPI4_RX_DMA_IRQ                DMAC2_CH7_IRQn

/* DMA2 channel8 */
#define I2C7_DMA_IRQHandler              DMAC2_CH8_IRQHandler
#define I2C7_DMA_IRQ_PRIO                1
#define I2C7_DMA_INSTANCE                DMA2_Channel8
#define I2C7_DMA_IRQ                     DMAC2_CH8_IRQn

/* DMA2 channel8 */
#define SPI4_DMA_TX_IRQHandler         DMAC2_CH8_IRQHandler
#define SPI4_TX_DMA_IRQ_PRIO           0
#define SPI4_TX_DMA_INSTANCE           DMA2_Channel8
#define SPI4_TX_DMA_IRQ                DMAC2_CH8_IRQn

/* DMA2 channel7 */
#define UART6_DMA_TX_IRQHandler          DMAC2_CH7_IRQHandler
#define UART6_TX_DMA_IRQ_PRIO            0
#define UART6_TX_DMA_INSTANCE            DMA2_Channel7
#define UART6_TX_DMA_IRQ                 DMAC2_CH7_IRQn

/* DMA2 channel8 */
#define UART6_DMA_RX_IRQHandler          DMAC2_CH8_IRQHandler
#define UART6_RX_DMA_IRQ_PRIO            0
#define UART6_RX_DMA_INSTANCE            DMA2_Channel8
#define UART6_RX_DMA_IRQ                 DMAC2_CH8_IRQn

/* DMA2 channel8 */
#define FLASH5_IRQHandler              DMAC2_CH8_IRQHandler
#define FLASH5_DMA_IRQ_PRIO            0
#define FLASH5_DMA_INSTANCE            DMA2_Channel8
#define FLASH5_DMA_IRQ                 DMAC2_CH8_IRQn

#ifdef __cplusplus
}
#endif

#endif /* __DMA_CONFIG_H__ */

