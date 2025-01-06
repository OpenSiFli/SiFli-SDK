/**
  ******************************************************************************
  * @file   bf0_hal_dma.h
  * @author Sifli software development team
  * @brief   Header file of DMA HAL module.
  * @attention
  ******************************************************************************
*/
/*
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

#ifndef __BF0_HAL_DMA_H
#define __BF0_HAL_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

#if defined(SOC_BF0_LCPU) && (defined(SF32LB55X) || defined(SF32LB58X) || defined(SF32LB52X))
#else
/* SF32LB55X, SF32LB58X and SF32LB52X LCPU doesn't support DMA channel dynamic allocation,
 * SF32LB52X LCPU uses DMA for RF register restore and DMA driver uses ROM implementation
 */
#define DMA_SUPPORT_DYN_CHANNEL_ALLOC
#endif /* SOC_BF0_LCPU && (SF32LB55X || SF32LB58X || SF32LB52X) */

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup DMA DMA
  * @brief DMA HAL module driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup DMA_Exported_Types DMA Exported Types
  * @{
  */

struct dma_config
{
    DMA_Channel_TypeDef *Instance;    /*!< DMA HW low layer handle */
    uint32_t dma_irq_prio;            /*!< DMA interrupt priority */
    IRQn_Type dma_irq;              /*!< DMA interrupt number */
    uint32_t request;            /*!< DMA request number */
};

/**
  * @brief  DMA Configuration Structure definition
  */
typedef struct
{
    uint32_t Request;                   /*!< Specifies the request selected for the specified channel.
                                           This parameter can be a value of @ref DMA_request */

    uint32_t Direction;                 /*!< Specifies if the data will be transferred from memory to peripheral,
                                           from memory to memory or from peripheral to memory.
                                           This parameter can be a value of @ref DMA_Data_transfer_direction */

    uint32_t PeriphInc;                 /*!< Specifies whether the Peripheral address register should be incremented or not.
                                           This parameter can be a value of @ref DMA_Peripheral_incremented_mode */

    uint32_t MemInc;                    /*!< Specifies whether the memory address register should be incremented or not.
                                           This parameter can be a value of @ref DMA_Memory_incremented_mode */

    uint32_t PeriphDataAlignment;       /*!< Specifies the Peripheral data width.
                                           This parameter can be a value of @ref DMA_Peripheral_data_size */

    uint32_t MemDataAlignment;          /*!< Specifies the Memory data width.
                                           This parameter can be a value of @ref DMA_Memory_data_size */

    uint32_t Mode;                      /*!< Specifies the operation mode of the DMAy Channelx.
                                           This parameter can be a value of @ref DMA_mode
                                           @note The circular buffer mode cannot be used if the memory-to-memory
                                                 data transfer is configured on the selected Channel */

    uint32_t Priority;                  /*!< Specifies the software priority for the DMAy Channelx.
                                           This parameter can be a value of @ref DMA_Priority_level */

    uint32_t BurstSize;                 /*!< Specifies the burst size in non-m2m mode
                                          When BS>1, DMA will transfer for BS times for each request if left NDT is larger than BS,
                                          or else transfer for left NDT times.
                                          When BS=0 or 1, DMA will always do single transfer for each request.
                                          In memory-to-memory mode, BS is ignored.*/
#ifdef DMA_SUPPORT_DYN_CHANNEL_ALLOC
    uint32_t IrqPrio;                   /*!< Interrupt Priority*/
#endif /* DMA_SUPPORT_DYN_CHANNEL_ALLOC */
} DMA_InitTypeDef;

/**
  * @brief  HAL DMA State structures definition
  */
typedef enum
{
    HAL_DMA_STATE_RESET             = 0x00U,  /*!< DMA not yet initialized or disabled    */
    HAL_DMA_STATE_READY             = 0x01U,  /*!< DMA initialized and ready for use      */
    HAL_DMA_STATE_BUSY              = 0x02U,  /*!< DMA process is ongoing                 */
    HAL_DMA_STATE_TIMEOUT           = 0x03U,  /*!< DMA timeout state                     */
} HAL_DMA_StateTypeDef;

/**
  * @brief  HAL DMA Error Code structure definition
  */
typedef enum
{
    HAL_DMA_FULL_TRANSFER      = 0x00U,    /*!< Full transfer     */
    HAL_DMA_HALF_TRANSFER      = 0x01U     /*!< Half Transfer     */
} HAL_DMA_LevelCompleteTypeDef;


/**
  * @brief  HAL DMA Callback ID structure definition
  */
typedef enum
{
    HAL_DMA_XFER_CPLT_CB_ID          = 0x00U,    /*!< Full transfer     */
    HAL_DMA_XFER_HALFCPLT_CB_ID      = 0x01U,    /*!< Half transfer     */
    HAL_DMA_XFER_ERROR_CB_ID         = 0x02U,    /*!< Error             */
    HAL_DMA_XFER_ABORT_CB_ID         = 0x03U,    /*!< Abort             */
    HAL_DMA_XFER_ALL_CB_ID           = 0x04U     /*!< All               */
} HAL_DMA_CallbackIDTypeDef;

/**
  * @brief  DMA handle Structure definition
  */
typedef struct __DMA_HandleTypeDef
{
    DMA_Channel_TypeDef    *Instance;                                                  /*!< Register base address                */

    DMA_InitTypeDef       Init;                                                        /*!< DMA communication parameters         */

    HAL_LockTypeDef       Lock;                                                        /*!< DMA locking object                   */

    __IO HAL_DMA_StateTypeDef  State;                                                  /*!< DMA transfer state                   */

    void                  *Parent;                                                     /*!< Parent object state                  */

    void (* XferCpltCallback)(struct __DMA_HandleTypeDef *hdma);                       /*!< DMA transfer complete callback       */

    void (* XferHalfCpltCallback)(struct __DMA_HandleTypeDef *hdma);                   /*!< DMA Half transfer complete callback  */

    void (* XferErrorCallback)(struct __DMA_HandleTypeDef *hdma);                      /*!< DMA transfer error callback          */

    void (* XferAbortCallback)(struct __DMA_HandleTypeDef *hdma);                      /*!< DMA transfer abort callback          */

    __IO uint32_t          ErrorCode;                                                  /*!< DMA Error code                       */

    DMAC_TypeDef           *DmaBaseAddress;                                            /*!< DMA Channel Base Address             */

    uint32_t               ChannelIndex;                                               /*!< DMA Channel Index                    */
#ifdef DMA_SUPPORT_DYN_CHANNEL_ALLOC
    uint32_t               LeftCounts;
    uint32_t               TotalCounts;
    uint32_t               TransCounts;
    uint32_t               SrcAddress;
    uint32_t               DstAddress;
    uint8_t                SrcInc;
    uint8_t                DstInc;
    uint8_t                SrcWidth;                                                   /*!< Src width, 0: 1 byte, 1: 2bytes, 2: 4bytes */
    uint8_t                DstWidth;                                                   /*!< Dst width, 0: 1 byte, 1: 2bytes, 2: 4bytes */
#endif /* DMA_SUPPORT_DYN_CHANNEL_ALLOC */
} DMA_HandleTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup DMA_Exported_Constants DMA Exported Constants
  * @ingroup DMA
  * @{
  */

/** @defgroup DMA_Error_Code DMA Error Code
  * @{
  */
#define HAL_DMA_ERROR_NONE                 0x00000000U    /*!< No error                                */
#define HAL_DMA_ERROR_TE                   0x00000001U    /*!< Transfer error                          */
#define HAL_DMA_ERROR_NO_XFER              0x00000004U    /*!< Abort requested with no Xfer ongoing    */
#define HAL_DMA_ERROR_TIMEOUT              0x00000020U    /*!< Timeout error                           */
#define HAL_DMA_ERROR_NOT_SUPPORTED        0x00000100U    /*!< Not supported mode                      */
#define HAL_DMA_ERROR_SYNC                 0x00000200U    /*!< DMAMUX sync overrun  error              */
#define HAL_DMA_ERROR_REQGEN               0x00000400U    /*!< DMAMUX request generator overrun  error */

/**
  * @}
  */

/** @defgroup DMA_request DMA request
  * @{
  */
#if !defined (DMAMUX1)

#define DMA_REQUEST_0                     0U
#define DMA_REQUEST_1                     1U
#define DMA_REQUEST_2                     2U
#define DMA_REQUEST_3                     3U
#define DMA_REQUEST_4                     4U
#define DMA_REQUEST_5                     5U
#define DMA_REQUEST_6                     6U
#define DMA_REQUEST_7                     7U
#define DMA_REQUEST_8                     8U
#define DMA_REQUEST_9                     9U
#define DMA_REQUEST_10                    10U
#define DMA_REQUEST_11                    11U
#define DMA_REQUEST_12                    12U
#define DMA_REQUEST_13                    13U
#define DMA_REQUEST_14                    14U
#define DMA_REQUEST_15                    15U
#define DMA_REQUEST_16                    16U
#define DMA_REQUEST_17                    17U
#define DMA_REQUEST_18                    18U
#define DMA_REQUEST_19                    19U
#define DMA_REQUEST_20                    20U
#define DMA_REQUEST_21                    21U
#define DMA_REQUEST_22                    22U
#define DMA_REQUEST_23                    23U
#define DMA_REQUEST_24                    24U
#define DMA_REQUEST_25                    25U
#define DMA_REQUEST_26                    26U
#define DMA_REQUEST_27                    27U
#define DMA_REQUEST_28                    28U
#define DMA_REQUEST_29                    29U
#define DMA_REQUEST_30                    30U
#define DMA_REQUEST_31                    31U
#if defined(SF32LB56X) || defined(SF32LB52X)
#define DMA_REQUEST_32                    32U
#define DMA_REQUEST_33                    33U
#define DMA_REQUEST_34                    34U
#define DMA_REQUEST_35                    35U
#define DMA_REQUEST_36                    36U
#define DMA_REQUEST_37                    37U
#define DMA_REQUEST_38                    38U
#define DMA_REQUEST_39                    39U
#define DMA_REQUEST_40                    40U
#define DMA_REQUEST_41                    41U
#define DMA_REQUEST_42                    42U
#define DMA_REQUEST_43                    43U
#define DMA_REQUEST_44                    44U
#define DMA_REQUEST_45                    45U
#define DMA_REQUEST_46                    46U
#define DMA_REQUEST_47                    47U
#define DMA_REQUEST_48                    48U
#define DMA_REQUEST_49                    49U
#define DMA_REQUEST_50                    50U
#define DMA_REQUEST_51                    51U
#define DMA_REQUEST_52                    52U
#define DMA_REQUEST_53                    53U
#define DMA_REQUEST_54                    54U
#define DMA_REQUEST_55                    55U
#define DMA_REQUEST_56                    56U
#define DMA_REQUEST_57                    57U
#define DMA_REQUEST_58                    58U
#define DMA_REQUEST_59                    59U
#define DMA_REQUEST_60                    60U
#define DMA_REQUEST_61                    61U
#define DMA_REQUEST_62                    62U
#define DMA_REQUEST_63                    63U
#endif /* SF32LB56X || SF32LB52X */


#endif

#define MAX_DMA_CHANNEL 8

#if defined(DMAMUX1)

#define DMA_REQUEST_MEM2MEM                 0U  /*!< memory to memory transfer   */

#define DMA_REQUEST_GENERATOR0              1U  /*!< DMAMUX1 request generator 0 */
#define DMA_REQUEST_GENERATOR1              2U  /*!< DMAMUX1 request generator 1 */
#define DMA_REQUEST_GENERATOR2              3U  /*!< DMAMUX1 request generator 2 */
#define DMA_REQUEST_GENERATOR3              4U  /*!< DMAMUX1 request generator 3 */

#define DMA_REQUEST_ADC1                    5U  /*!< DMAMUX1 ADC1 request      */

#define DMA_REQUEST_DAC1_CH1                6U  /*!< DMAMUX1 DAC1 CH1 request  */
#define DMA_REQUEST_DAC1_CH2                7U  /*!< DMAMUX1 DAC1 CH2 request  */

#define DMA_REQUEST_TIM6_UP                 8U  /*!< DMAMUX1 TIM6 UP request   */
#define DMA_REQUEST_TIM7_UP                 9U  /*!< DMAMUX1 TIM7 UP request   */

#define DMA_REQUEST_SPI1_RX                10U  /*!< DMAMUX1 SPI1 RX request   */
#define DMA_REQUEST_SPI1_TX                11U  /*!< DMAMUX1 SPI1 TX request   */
#define DMA_REQUEST_SPI2_RX                12U  /*!< DMAMUX1 SPI2 RX request   */
#define DMA_REQUEST_SPI2_TX                13U  /*!< DMAMUX1 SPI2 TX request   */
#define DMA_REQUEST_SPI3_RX                14U  /*!< DMAMUX1 SPI3 RX request   */
#define DMA_REQUEST_SPI3_TX                15U  /*!< DMAMUX1 SPI3 TX request   */

#define DMA_REQUEST_I2C1_RX                16U  /*!< DMAMUX1 I2C1 RX request   */
#define DMA_REQUEST_I2C1_TX                17U  /*!< DMAMUX1 I2C1 TX request   */
#define DMA_REQUEST_I2C2_RX                18U  /*!< DMAMUX1 I2C2 RX request   */
#define DMA_REQUEST_I2C2_TX                19U  /*!< DMAMUX1 I2C2 TX request   */
#define DMA_REQUEST_I2C3_RX                20U  /*!< DMAMUX1 I2C3 RX request   */
#define DMA_REQUEST_I2C3_TX                21U  /*!< DMAMUX1 I2C3 TX request   */
#define DMA_REQUEST_I2C4_RX                22U  /*!< DMAMUX1 I2C4 RX request   */
#define DMA_REQUEST_I2C4_TX                23U  /*!< DMAMUX1 I2C4 TX request   */

#define DMA_REQUEST_USART1_RX              24U  /*!< DMAMUX1 USART1 RX request */
#define DMA_REQUEST_USART1_TX              25U  /*!< DMAMUX1 USART1 TX request */
#define DMA_REQUEST_USART2_RX              26U  /*!< DMAMUX1 USART2 RX request */
#define DMA_REQUEST_USART2_TX              27U  /*!< DMAMUX1 USART2 TX request */
#define DMA_REQUEST_USART3_RX              28U  /*!< DMAMUX1 USART3 RX request */
#define DMA_REQUEST_USART3_TX              29U  /*!< DMAMUX1 USART3 TX request */

#define DMA_REQUEST_UART4_RX               30U  /*!< DMAMUX1 UART4 RX request  */
#define DMA_REQUEST_UART4_TX               31U  /*!< DMAMUX1 UART4 TX request  */
#define DMA_REQUEST_UART5_RX               32U  /*!< DMAMUX1 UART5 RX request  */
#define DMA_REQUEST_UART5_TX               33U  /*!< DMAMUX1 UART5 TX request  */

#define DMA_REQUEST_LPUART1_RX             34U  /*!< DMAMUX1 LP_UART1_RX request */
#define DMA_REQUEST_LPUART1_TX             35U  /*!< DMAMUX1 LP_UART1_RX request */

#define DMA_REQUEST_SAI1_A                 36U  /*!< DMAMUX1 SAI1 A request    */
#define DMA_REQUEST_SAI1_B                 37U  /*!< DMAMUX1 SAI1 B request    */
#define DMA_REQUEST_SAI2_A                 38U  /*!< DMAMUX1 SAI2 A request    */
#define DMA_REQUEST_SAI2_B                 39U  /*!< DMAMUX1 SAI2 B request    */

#define DMA_REQUEST_OCTOSPI1               40U  /*!< DMAMUX1 OCTOSPI1 request  */
#define DMA_REQUEST_OCTOSPI2               41U  /*!< DMAMUX1 OCTOSPI2 request  */

#define DMA_REQUEST_TIM1_CH1               42U  /*!< DMAMUX1 TIM1 CH1 request  */
#define DMA_REQUEST_TIM1_CH2               43U  /*!< DMAMUX1 TIM1 CH2 request  */
#define DMA_REQUEST_TIM1_CH3               44U  /*!< DMAMUX1 TIM1 CH3 request  */
#define DMA_REQUEST_TIM1_CH4               45U  /*!< DMAMUX1 TIM1 CH4 request  */
#define DMA_REQUEST_TIM1_UP                46U  /*!< DMAMUX1 TIM1 UP  request  */
#define DMA_REQUEST_TIM1_TRIG              47U  /*!< DMAMUX1 TIM1 TRIG request */
#define DMA_REQUEST_TIM1_COM               48U  /*!< DMAMUX1 TIM1 COM request  */

#define DMA_REQUEST_TIM8_CH1               49U  /*!< DMAMUX1 TIM8 CH1 request  */
#define DMA_REQUEST_TIM8_CH2               50U  /*!< DMAMUX1 TIM8 CH2 request  */
#define DMA_REQUEST_TIM8_CH3               51U  /*!< DMAMUX1 TIM8 CH3 request  */
#define DMA_REQUEST_TIM8_CH4               52U  /*!< DMAMUX1 TIM8 CH4 request  */
#define DMA_REQUEST_TIM8_UP                53U  /*!< DMAMUX1 TIM8 UP  request  */
#define DMA_REQUEST_TIM8_TRIG              54U  /*!< DMAMUX1 TIM8 TRIG request */
#define DMA_REQUEST_TIM8_COM               55U  /*!< DMAMUX1 TIM8 COM request  */

#define DMA_REQUEST_TIM2_CH1               56U  /*!< DMAMUX1 TIM2 CH1 request  */
#define DMA_REQUEST_TIM2_CH2               57U  /*!< DMAMUX1 TIM2 CH2 request  */
#define DMA_REQUEST_TIM2_CH3               58U  /*!< DMAMUX1 TIM2 CH3 request  */
#define DMA_REQUEST_TIM2_CH4               59U  /*!< DMAMUX1 TIM2 CH4 request  */
#define DMA_REQUEST_TIM2_UP                60U  /*!< DMAMUX1 TIM2 UP  request  */

#define DMA_REQUEST_TIM3_CH1               61U  /*!< DMAMUX1 TIM3 CH1 request  */
#define DMA_REQUEST_TIM3_CH2               62U  /*!< DMAMUX1 TIM3 CH2 request  */
#define DMA_REQUEST_TIM3_CH3               63U  /*!< DMAMUX1 TIM3 CH3 request  */
#define DMA_REQUEST_TIM3_CH4               64U  /*!< DMAMUX1 TIM3 CH4 request  */
#define DMA_REQUEST_TIM3_UP                65U  /*!< DMAMUX1 TIM3 UP  request  */
#define DMA_REQUEST_TIM3_TRIG              66U  /*!< DMAMUX1 TIM3 TRIG request */

#define DMA_REQUEST_TIM4_CH1               67U  /*!< DMAMUX1 TIM4 CH1 request  */
#define DMA_REQUEST_TIM4_CH2               68U  /*!< DMAMUX1 TIM4 CH2 request  */
#define DMA_REQUEST_TIM4_CH3               69U  /*!< DMAMUX1 TIM4 CH3 request  */
#define DMA_REQUEST_TIM4_CH4               70U  /*!< DMAMUX1 TIM4 CH4 request  */
#define DMA_REQUEST_TIM4_UP                71U  /*!< DMAMUX1 TIM4 UP  request  */

#define DMA_REQUEST_TIM5_CH1               72U  /*!< DMAMUX1 TIM5 CH1 request  */
#define DMA_REQUEST_TIM5_CH2               73U  /*!< DMAMUX1 TIM5 CH2 request  */
#define DMA_REQUEST_TIM5_CH3               74U  /*!< DMAMUX1 TIM5 CH3 request  */
#define DMA_REQUEST_TIM5_CH4               75U  /*!< DMAMUX1 TIM5 CH4 request  */
#define DMA_REQUEST_TIM5_UP                76U  /*!< DMAMUX1 TIM5 UP  request  */
#define DMA_REQUEST_TIM5_TRIG              77U  /*!< DMAMUX1 TIM5 TRIG request */

#define DMA_REQUEST_TIM15_CH1              78U  /*!< DMAMUX1 TIM15 CH1 request */
#define DMA_REQUEST_TIM15_UP               79U  /*!< DMAMUX1 TIM15 UP  request */
#define DMA_REQUEST_TIM15_TRIG             80U  /*!< DMAMUX1 TIM15 TRIG request */
#define DMA_REQUEST_TIM15_COM              81U  /*!< DMAMUX1 TIM15 COM request */

#define DMA_REQUEST_TIM16_CH1              82U  /*!< DMAMUX1 TIM16 CH1 request */
#define DMA_REQUEST_TIM16_UP               83U  /*!< DMAMUX1 TIM16 UP  request */
#define DMA_REQUEST_TIM17_CH1              84U  /*!< DMAMUX1 TIM17 CH1 request */
#define DMA_REQUEST_TIM17_UP               85U  /*!< DMAMUX1 TIM17 UP  request */

#define DMA_REQUEST_DFSDM1_FLT0            86U  /*!< DMAMUX1 DFSDM1 Filter0 request */
#define DMA_REQUEST_DFSDM1_FLT1            87U  /*!< DMAMUX1 DFSDM1 Filter1 request */
#define DMA_REQUEST_DFSDM1_FLT2            88U  /*!< DMAMUX1 DFSDM1 Filter2 request */
#define DMA_REQUEST_DFSDM1_FLT3            89U  /*!< DMAMUX1 DFSDM1 Filter3 request */

#define DMA_REQUEST_DCMI                   90U  /*!< DMAMUX1 DCMI request      */

#define DMA_REQUEST_AES_IN                 91U  /*!< DMAMUX1 AES IN request    */
#define DMA_REQUEST_AES_OUT                92U  /*!< DMAMUX1 AES OUT request   */

#define DMA_REQUEST_HASH_IN                93U  /*!< DMAMUX1 HASH IN request   */

#endif /* DMAMUX1 */

/**
  * @}  DMA Requst
  */

/** @defgroup DMA_Data_transfer_direction DMA Data transfer direction
  * @{
  */
#define DMA_PERIPH_TO_MEMORY         0x00000000U        /*!< Peripheral to memory direction */
#define DMA_MEMORY_TO_PERIPH         DMAC_CCR1_DIR        /*!< Memory to peripheral direction */
#define DMA_MEMORY_TO_MEMORY         DMAC_CCR1_MEM2MEM    /*!< Memory to memory direction     */
/**
  * @} DMA_Data_transfer_direction
  */

/** @defgroup DMA_Peripheral_incremented_mode DMA Peripheral incremented mode
  * @{
  */
#define DMA_PINC_ENABLE              DMAC_CCR1_PINC  /*!< Peripheral increment mode Enable */
#define DMA_PINC_DISABLE             0x00000000U   /*!< Peripheral increment mode Disable */
/**
  * @} DMA_Peripheral_incremented_mode
  */

/** @defgroup DMA_Memory_incremented_mode DMA Memory incremented mode
  * @{
  */
#define DMA_MINC_ENABLE              DMAC_CCR1_MINC   /*!< Memory increment mode Enable  */
#define DMA_MINC_DISABLE             0x00000000U    /*!< Memory increment mode Disable */
/**
  * @} DMA_Memory_incremented_mode
  */

/** @defgroup DMA_Peripheral_data_size DMA Peripheral data size
  * @{
  */
#define DMA_PDATAALIGN_BYTE          0x00000000U     /*!< Peripheral data alignment : Byte     */
#define DMA_PDATAALIGN_HALFWORD      DMAC_CCR1_PSIZE_0 /*!< Peripheral data alignment : HalfWord */
#define DMA_PDATAALIGN_WORD          DMAC_CCR1_PSIZE_1 /*!< Peripheral data alignment : Word     */
/**
  * @} DMA_Peripheral_data_size
  */

/** @defgroup DMA_Memory_data_size DMA Memory data size
  * @{
  */
#define DMA_MDATAALIGN_BYTE          0x00000000U     /*!< Memory data alignment : Byte     */
#define DMA_MDATAALIGN_HALFWORD      DMAC_CCR1_MSIZE_0 /*!< Memory data alignment : HalfWord */
#define DMA_MDATAALIGN_WORD          DMAC_CCR1_MSIZE_1 /*!< Memory data alignment : Word     */
/**
  * @} DMA_Memory_data_size
  */

/** @defgroup DMA_mode DMA mode
  * @{
  */
#define DMA_NORMAL                   0x00000000U     /*!< Normal mode                  */
#define DMA_CIRCULAR                 DMAC_CCR1_CIRC    /*!< Circular mode                */
/**
  * @} DMA_mode
  */

/** @defgroup DMA_Priority_level DMA Priority level
  * @{
  */
#define DMA_PRIORITY_LOW             0x00000000U            /*!< Priority level : Low       */
#define DMA_PRIORITY_MEDIUM          DMAC_CCR1_PL_0     /*!< Priority level : Medium    */
#define DMA_PRIORITY_HIGH            DMAC_CCR1_PL_1     /*!< Priority level : High      */
#define DMA_PRIORITY_VERY_HIGH       DMAC_CCR1_PL       /*!< Priority level : Very_High */
/**
  * @} DMA_Priority_level
  */


/** @defgroup DMA_interrupt_enable_definitions DMA interrupt enable definitions
  * @{
  */
#define DMA_IT_TC                         DMAC_CCR1_TCIE
#define DMA_IT_HT                         DMAC_CCR1_HTIE
#define DMA_IT_TE                         DMAC_CCR1_TEIE
/**
  * @} DMA_interrupt_enable_definitions
  */

/** @defgroup DMA_flag_definitions DMA flag definitions
  * @{
  */
#define DMA_FLAG_GL1                      DMAC_ISR_GIF1
#define DMA_FLAG_TC1                      DMAC_ISR_TCIF1
#define DMA_FLAG_HT1                      DMAC_ISR_HTIF1
#define DMA_FLAG_TE1                      DMAC_ISR_TEIF1
#define DMA_FLAG_GL2                      DMAC_ISR_GIF2
#define DMA_FLAG_TC2                      DMAC_ISR_TCIF2
#define DMA_FLAG_HT2                      DMAC_ISR_HTIF2
#define DMA_FLAG_TE2                      DMAC_ISR_TEIF2
#define DMA_FLAG_GL3                      DMAC_ISR_GIF3
#define DMA_FLAG_TC3                      DMAC_ISR_TCIF3
#define DMA_FLAG_HT3                      DMAC_ISR_HTIF3
#define DMA_FLAG_TE3                      DMAC_ISR_TEIF3
#define DMA_FLAG_GL4                      DMAC_ISR_GIF4
#define DMA_FLAG_TC4                      DMAC_ISR_TCIF4
#define DMA_FLAG_HT4                      DMAC_ISR_HTIF4
#define DMA_FLAG_TE4                      DMAC_ISR_TEIF4
#define DMA_FLAG_GL5                      DMAC_ISR_GIF5
#define DMA_FLAG_TC5                      DMAC_ISR_TCIF5
#define DMA_FLAG_HT5                      DMAC_ISR_HTIF5
#define DMA_FLAG_TE5                      DMAC_ISR_TEIF5
#define DMA_FLAG_GL6                      DMAC_ISR_GIF6
#define DMA_FLAG_TC6                      DMAC_ISR_TCIF6
#define DMA_FLAG_HT6                      DMAC_ISR_HTIF6
#define DMA_FLAG_TE6                      DMAC_ISR_TEIF6
#define DMA_FLAG_GL7                      DMAC_ISR_GIF7
#define DMA_FLAG_TC7                      DMAC_ISR_TCIF7
#define DMA_FLAG_HT7                      DMAC_ISR_HTIF7
#define DMA_FLAG_TE7                      DMAC_ISR_TEIF7

/**
  * @} DMA_flag_definitions
  */

/**
  * @} DMA_Exported_Constants
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup DMA_Exported_Macros DMA Exported Macros
  * @ingroup DMA
  * @{
  */

/** @brief  Reset DMA handle state.
  * @param  \__HANDLE__: DMA handle
  * @retval None
  */
#define __HAL_DMA_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_DMA_STATE_RESET)

/**
  * @brief  Enable the specified DMA Channel.
  * @param  \__HANDLE__: DMA handle
  * @retval None
  */
#define __HAL_DMA_ENABLE(__HANDLE__)        ((__HANDLE__)->Instance->CCR |=  DMAC_CCR1_EN)

/**
  * @brief  Disable the specified DMA Channel.
  * @param  \__HANDLE__: DMA handle
  * @retval None
  */
#define __HAL_DMA_DISABLE(__HANDLE__)       ((__HANDLE__)->Instance->CCR &=  ~DMAC_CCR1_EN)


/* Interrupt & Flag management */

/**
  * @brief  Return the current DMA Channel transfer complete flag.
  * @param  \__HANDLE__: DMA handle
  * @retval The specified transfer complete flag index.
  */

#define __HAL_DMA_GET_TC_FLAG_INDEX(__HANDLE__) \
(((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel1))? DMA_FLAG_TC1 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel1))? DMA_FLAG_TC1 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel2))? DMA_FLAG_TC2 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel2))? DMA_FLAG_TC2 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel3))? DMA_FLAG_TC3 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel3))? DMA_FLAG_TC3 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel4))? DMA_FLAG_TC4 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel4))? DMA_FLAG_TC4 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel5))? DMA_FLAG_TC5 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel5))? DMA_FLAG_TC5 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel6))? DMA_FLAG_TC6 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel6))? DMA_FLAG_TC6 :\
   DMA_FLAG_TC7)

/**
  * @brief  Return the current DMA Channel half transfer complete flag.
  * @param  \__HANDLE__: DMA handle
  * @retval The specified half transfer complete flag index.
  */
#define __HAL_DMA_GET_HT_FLAG_INDEX(__HANDLE__)\
(((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel1))? DMA_FLAG_HT1 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel1))? DMA_FLAG_HT1 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel2))? DMA_FLAG_HT2 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel2))? DMA_FLAG_HT2 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel3))? DMA_FLAG_HT3 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel3))? DMA_FLAG_HT3 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel4))? DMA_FLAG_HT4 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel4))? DMA_FLAG_HT4 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel5))? DMA_FLAG_HT5 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel5))? DMA_FLAG_HT5 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel6))? DMA_FLAG_HT6 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel6))? DMA_FLAG_HT6 :\
   DMA_FLAG_HT7)

/**
  * @brief  Return the current DMA Channel transfer error flag.
  * @param  \__HANDLE__: DMA handle
  * @retval The specified transfer error flag index.
  */
#define __HAL_DMA_GET_TE_FLAG_INDEX(__HANDLE__)\
(((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel1))? DMA_FLAG_TE1 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel1))? DMA_FLAG_TE1 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel2))? DMA_FLAG_TE2 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel2))? DMA_FLAG_TE2 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel3))? DMA_FLAG_TE3 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel3))? DMA_FLAG_TE3 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel4))? DMA_FLAG_TE4 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel4))? DMA_FLAG_TE4 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel5))? DMA_FLAG_TE5 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel5))? DMA_FLAG_TE5 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel6))? DMA_FLAG_TE6 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel6))? DMA_FLAG_TE6 :\
   DMA_FLAG_TE7)

/**
  * @brief  Return the current DMA Channel Global interrupt flag.
  * @param  \__HANDLE__: DMA handle
  * @retval The specified transfer error flag index.
  */
#define __HAL_DMA_GET_GI_FLAG_INDEX(__HANDLE__)\
(((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel1))? DMAC_ISR_GIF1 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel1))? DMAC_ISR_GIF1 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel2))? DMAC_ISR_GIF2 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel2))? DMAC_ISR_GIF2 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel3))? DMAC_ISR_GIF3 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel3))? DMAC_ISR_GIF3 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel4))? DMAC_ISR_GIF4 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel4))? DMAC_ISR_GIF4 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel5))? DMAC_ISR_GIF5 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel5))? DMAC_ISR_GIF5 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA1_Channel6))? DMAC_ISR_GIF6 :\
 ((uint32_t)((__HANDLE__)->Instance) == ((uint32_t)DMA2_Channel6))? DMAC_ISR_GIF6 :\
   DMAC_ISR_GIF7)


/**
  * @brief  Get the DMA Channel pending flags.
  * @param  \__HANDLE__: DMA handle
  * @param  \__FLAG__: Get the specified flag.
  *          This parameter can be any combination of the following values:
  *            @arg DMA_FLAG_TCx:  Transfer complete flag
  *            @arg DMA_FLAG_HTx:  Half transfer complete flag
  *            @arg DMA_FLAG_TEx:  Transfer error flag
  *            @arg DMA_FLAG_GLx:  Global interrupt flag
  *         Where x can be from 1 to 7 to select the DMA Channel x flag.
  * @retval The state of FLAG (SET or RESET).
  */
#define __HAL_DMA_GET_FLAG(__HANDLE__, __FLAG__) ((uint32_t)((__HANDLE__)->DmaBaseAddress->ISR & (__FLAG__:)))

/**
  * @brief  Clear the DMA Channel pending flags.
  * @param  \__HANDLE__: DMA handle
  * @param  \__FLAG__: specifies the flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg DMA_FLAG_TCx:  Transfer complete flag
  *            @arg DMA_FLAG_HTx:  Half transfer complete flag
  *            @arg DMA_FLAG_TEx:  Transfer error flag
  *            @arg DMA_FLAG_GLx:  Global interrupt flag
  *         Where x can be from 1 to 7 to select the DMA Channel x flag.
  * @retval None
  */
#define __HAL_DMA_CLEAR_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->DmaBaseAddress->IFCR = (__FLAG__))

/**
  * @brief  Enable the specified DMA Channel interrupts.
  * @param  \__HANDLE__: DMA handle
  * @param \__INTERRUPT__: specifies the DMA interrupt sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg DMA_IT_TC:  Transfer complete interrupt mask
  *            @arg DMA_IT_HT:  Half transfer complete interrupt mask
  *            @arg DMA_IT_TE:  Transfer error interrupt mask
  * @retval None
  */
#define __HAL_DMA_ENABLE_IT(__HANDLE__, __INTERRUPT__)   ((__HANDLE__)->Instance->CCR |= (__INTERRUPT__))

/**
  * @brief  Disable the specified DMA Channel interrupts.
  * @param  \__HANDLE__: DMA handle
  * @param \__INTERRUPT__: specifies the DMA interrupt sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg DMA_IT_TC:  Transfer complete interrupt mask
  *            @arg DMA_IT_HT:  Half transfer complete interrupt mask
  *            @arg DMA_IT_TE:  Transfer error interrupt mask
  * @retval None
  */
#define __HAL_DMA_DISABLE_IT(__HANDLE__, __INTERRUPT__)  ((__HANDLE__)->Instance->CCR &= ~(__INTERRUPT__))

/**
  * @brief  Check whether the specified DMA Channel interrupt is enabled or not.
  * @param  \__HANDLE__: DMA handle
  * @param  \__INTERRUPT__: specifies the DMA interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg DMA_IT_TC:  Transfer complete interrupt mask
  *            @arg DMA_IT_HT:  Half transfer complete interrupt mask
  *            @arg DMA_IT_TE:  Transfer error interrupt mask
  * @retval The state of DMA_IT (SET or RESET).
  */
#define __HAL_DMA_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)  (((__HANDLE__)->Instance->CCR & (__INTERRUPT__)))

/**
  * @brief  Return the number of remaining data units in the current DMA Channel transfer.
  * @param  \__HANDLE__: DMA handle
  * @retval The number of remaining data units in the current DMA Channel transfer.
  */
#define __HAL_DMA_GET_COUNTER(__HANDLE__) ((__HANDLE__)->Instance->CNDTR)

/**
  * @brief  restart the current DMA Channel transfer.
  * @param  \__HANDLE__: DMA handle
  * @param  \__DATA_COUNTS__: number of data to transfer (0 to 2^16 - 1)
  * @retval None.
  */
#define __HAL_DMA_SET_COUNTER(__HANDLE__, __DATA_COUNTS__) ((__HANDLE__)->Instance->CNDTR = __DATA_COUNTS__)

/**
  * @brief  Set circular mode for the current DMA Channel transfer.
  * @param  \__HANDLE__: DMA handle
  * @param  \__CIRCLUAR_MODE__: Circluar mode, 0 disable, 1 enable
  * @retval None.
  */
#define __HAL_DMA_SET_CIRCLUAR_MODE(__HANDLE__, __CIRCLUAR_MODE__) ((__HANDLE__)->Instance->CCR |= __CIRCLUAR_MODE__)

/**
  * @brief  Clear circular mode for the current DMA Channel transfer.
  * @param  \__HANDLE__: DMA handle
  * @param  \__CIRCLUAR_MODE__: Circluar mode, 0 disable, 1 enable
  * @retval None.
  */
#define __HAL_DMA_CLEAR_CIRCLUAR_MODE(__HANDLE__, __CIRCLUAR_MODE__) ((__HANDLE__)->Instance->CCR &= ~(__CIRCLUAR_MODE__))


/**
  * @}
  */

#if defined(DMAMUX1)
/* Include DMA HAL Extension module */
#include "bf0_hal_dma_ex.h"
#endif /* DMAMUX1 */

/* Exported functions --------------------------------------------------------*/

/** @defgroup DMA_Exported_Functions DMA Exported Functions
  * @{
  */

/** @defgroup DMA_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief   Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
             ##### Initialization and de-initialization functions  #####
 ===============================================================================
    [..]
    This section provides functions allowing to initialize the DMA Channel source
    and destination addresses, incrementation and data sizes, transfer direction,
    circular/normal mode selection, memory-to-memory mode selection and Channel priority value.
    [..]
    The HAL_DMA_Init() function follows the DMA configuration procedures as described in
    reference manual.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the DMA according to the specified
  *         parameters in the DMA_InitTypeDef and initialize the associated handle.
  * @param  hdma Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);

/**
  * @brief  DeInitialize the DMA peripheral.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma);
/**
  * @} DMA_Exported_Functions_Group1
  */

/** @defgroup DMA_Exported_Functions_Group2 Input and Output operation functions
 *  @brief   Input and Output operation functions
 *
@verbatim
 ===============================================================================
                      #####  IO operation functions  #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure the source, destination address and data length and Start DMA transfer
      (+) Configure the source, destination address and data length and
          Start DMA transfer with interrupt
      (+) Abort DMA transfer
      (+) Poll for transfer complete
      (+) Handle DMA interrupt request

@endverbatim
  * @{
  */

/**
  * @brief  Start the DMA Transfer.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  Counts The counts of data transfer action
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Start(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t Counts);

/**
  * @brief  Start the DMA Transfer with interrupt enabled.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  Counts The counts of data transfer action
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t Counts);
/**
  * @brief  Abort the DMA Transfer.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
    * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);

/**
  * @brief  Aborts the DMA Transfer in Interrupt mode.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                 the configuration information for the specified DMA Channel.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma);

/**
  * @brief  Polling for transfer complete.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                  the configuration information for the specified DMA Channel.
  * @param  CompleteLevel Specifies the DMA level complete.
  * @param  Timeout       Timeout duration.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, HAL_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout);

/**
  * @brief  Handle DMA interrupt request.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @retval None
  */
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);

#ifdef DMA_SUPPORT_DYN_CHANNEL_ALLOC
void HAL_DMAC1_CH1_IRQHandler(void);

void HAL_DMAC1_CH2_IRQHandler(void);

void HAL_DMAC1_CH3_IRQHandler(void);

void HAL_DMAC1_CH4_IRQHandler(void);

void HAL_DMAC1_CH5_IRQHandler(void);

void HAL_DMAC1_CH6_IRQHandler(void);

void HAL_DMAC1_CH7_IRQHandler(void);

void HAL_DMAC1_CH8_IRQHandler(void);

void HAL_DMAC2_CH1_IRQHandler(void);

void HAL_DMAC2_CH2_IRQHandler(void);

void HAL_DMAC2_CH3_IRQHandler(void);

void HAL_DMAC2_CH4_IRQHandler(void);

void HAL_DMAC2_CH5_IRQHandler(void);

void HAL_DMAC2_CH6_IRQHandler(void);

void HAL_DMAC2_CH7_IRQHandler(void);

void HAL_DMAC2_CH8_IRQHandler(void);

#ifdef DMA3
void HAL_DMAC3_CH1_IRQHandler(void);

void HAL_DMAC3_CH2_IRQHandler(void);

void HAL_DMAC3_CH3_IRQHandler(void);

void HAL_DMAC3_CH4_IRQHandler(void);

void HAL_DMAC3_CH5_IRQHandler(void);

void HAL_DMAC3_CH6_IRQHandler(void);

void HAL_DMAC3_CH7_IRQHandler(void);

void HAL_DMAC3_CH8_IRQHandler(void);
#endif /* DMA3 */
#endif /* DMA_SUPPORT_DYN_CHANNEL_ALLOC */

/**
  * @brief  Register callbacks
  * @param  hdma                 pointer to a DMA_HandleTypeDef structure that contains
  *                               the configuration information for the specified DMA Channel.
  * @param  CallbackID           User Callback identifer
  *                               a HAL_DMA_CallbackIDTypeDef ENUM as parameter.
  * @param  pCallback            pointer to private callbacsk function which has pointer to
  *                               a DMA_HandleTypeDef structure as parameter.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)(DMA_HandleTypeDef *_hdma));

/**
  * @brief  UnRegister callbacks
  * @param  hdma                 pointer to a DMA_HandleTypeDef structure that contains
  *                               the configuration information for the specified DMA Channel.
  * @param  CallbackID           User Callback identifer
  *                               a HAL_DMA_CallbackIDTypeDef ENUM as parameter.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID);
void HAL_DMA_Select_Source(DMA_Channel_TypeDef *dma_chl, uint8_t request);

/**
  * @} DMA_Exported_Functions_Group2
  */



/** @defgroup DMA_Exported_Functions_Group3 Peripheral State and Errors functions
 *  @brief    Peripheral State and Errors functions
 *
@verbatim
 ===============================================================================
            ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Check the DMA state
      (+) Get error code

@endverbatim
  * @{
  */

/**
  * @brief  Return the DMA hande state.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @retval HAL state
  */
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);

/**
  * @brief  Return the DMA error code.
  * @param  hdma : pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA Channel.
  * @retval DMA Error Code
  */
uint32_t             HAL_DMA_GetError(DMA_HandleTypeDef *hdma);
/**
  * @} DMA_Exported_Functions_Group3
  */

/**
  * @} DMA_Exported_Functions
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup DMA_Private_Macros DMA Private Macros
  * @{
  */

#define IS_DMA_DIRECTION(DIRECTION) (((DIRECTION) == DMA_PERIPH_TO_MEMORY ) || \
                                     ((DIRECTION) == DMA_MEMORY_TO_PERIPH)  || \
                                     ((DIRECTION) == DMA_MEMORY_TO_MEMORY))

#define IS_DMA_BUFFER_SIZE(SIZE) (((SIZE) >= 0x1U) && ((SIZE) < 0x10000U))

#define IS_DMA_PERIPHERAL_INC_STATE(STATE) (((STATE) == DMA_PINC_ENABLE) || \
                                            ((STATE) == DMA_PINC_DISABLE))

#define IS_DMA_MEMORY_INC_STATE(STATE) (((STATE) == DMA_MINC_ENABLE)  || \
                                        ((STATE) == DMA_MINC_DISABLE))

#if !defined (DMAMUX1)
#if defined(SF32LB56X) || defined(SF32LB52X)
#define IS_DMA_ALL_REQUEST(REQUEST) ((REQUEST) <= DMA_REQUEST_63)
#else
#define IS_DMA_ALL_REQUEST(REQUEST) ((REQUEST) <= DMA_REQUEST_31)
#endif
#endif

#if defined(DMAMUX1)

#define IS_DMA_ALL_REQUEST(REQUEST)((REQUEST) <= DMA_REQUEST_HASH_IN)

#endif /* DMAMUX1 */

#define IS_DMA_PERIPHERAL_DATA_SIZE(SIZE) (((SIZE) == DMA_PDATAALIGN_BYTE)     || \
                                           ((SIZE) == DMA_PDATAALIGN_HALFWORD) || \
                                           ((SIZE) == DMA_PDATAALIGN_WORD))

#define IS_DMA_MEMORY_DATA_SIZE(SIZE) (((SIZE) == DMA_MDATAALIGN_BYTE)     || \
                                       ((SIZE) == DMA_MDATAALIGN_HALFWORD) || \
                                       ((SIZE) == DMA_MDATAALIGN_WORD ))

#define IS_DMA_MODE(MODE) (((MODE) == DMA_NORMAL )  || \
                           ((MODE) == DMA_CIRCULAR))

#define IS_DMA_PRIORITY(PRIORITY) (((PRIORITY) == DMA_PRIORITY_LOW )   || \
                                   ((PRIORITY) == DMA_PRIORITY_MEDIUM) || \
                                   ((PRIORITY) == DMA_PRIORITY_HIGH)   || \
                                   ((PRIORITY) == DMA_PRIORITY_VERY_HIGH))

/* TODO: Revisit with real define*/
#define IS_DMA_ALL_INSTANCE(__INSTANCE__) (1)

#define DMA_1M_BOUNDARY_MASK      (0x000FFFFFUL)
#define DMA_1M_LEN                (0x00100000UL)
#define IS_DMA_ACCROSS_1M_BOUNDARY(addr,size) ((addr&(~DMA_1M_BOUNDARY_MASK))!=((addr+size-1)&(~DMA_1M_BOUNDARY_MASK)))

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_DMA_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

