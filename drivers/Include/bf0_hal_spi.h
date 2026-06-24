/*
 * SPDX-FileCopyrightText: 2016 STMicroelectronics
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
 */

#ifndef __BF0_HAL_SPI_H
#define __BF0_HAL_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup SPI SPI
  * @brief SPI HAL module driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup SPI_Exported_Types SPI Exported Types
  * @{
  */

/**
  * @brief  SPI Configuration Structure definition
  */
typedef struct
{
    uint32_t Mode;                /*!< Specifies the SPI operating mode.
                                     This parameter can be a value of @ref SPI_Mode */

    uint32_t Direction;           /*!< Specifies the SPI bidirectional mode state.
                                     This parameter can be a value of @ref SPI_Direction */

    uint32_t DataSize;            /*!< Specifies the SPI data size.
                                     This parameter can be a value of @ref SPI_Data_Size */

    uint32_t CLKPolarity;         /*!< Specifies the serial clock steady state.
                                     This parameter can be a value of @ref SPI_Clock_Polarity */

    uint32_t CLKPhase;            /*!< Specifies the clock active edge for the bit capture.
                                     This parameter can be a value of @ref SPI_Clock_Phase */



    uint32_t BaudRatePrescaler;   /*!< Specifies the Baud Rate prescaler value which will be
                                     used to configure the transmit and receive SCK clock.
                                     @note The communication clock is derived from the master
                                     clock. The slave clock does not need to be set. */


    uint32_t FrameFormat;         /*!< Specifies the data format.
                                     This parameter can be a value of @ref SPI_Frame_Format */

    uint32_t SFRMPol;             /*!< Specifies if the SPI mode frame synchronous signal (SFRMOUT / SFRMIN ) high or low.
                                     This parameter can be a value of @ref SPI_SFRM_POL */
#if 0
    uint32_t FirstBit;            /*!< Specifies whether data transfers start from MSB or LSB bit.
                                     This parameter can be a value of @ref SPI_MSB_LSB_transmission */

    uint32_t CRCCalculation;      /*!< Specifies if the CRC calculation is enabled or not.
                                     This parameter can be a value of @ref SPI_CRC_Calculation */

    uint32_t CRCPolynomial;       /*!< Specifies the polynomial used for the CRC calculation.
                                     This parameter must be an odd number between Min_Data = 1 and Max_Data = 65535 */

    uint32_t CRCLength;           /*!< Specifies the CRC Length used for the CRC calculation.
                                     CRC Length is only used with Data8 and Data16, not other data size
                                     This parameter can be a value of @ref SPI_CRC_length */

    uint32_t NSS;                 /*!< Specifies whether the NSS signal is managed by
                                     hardware (NSS pin) or by software using the SSI bit.
                                     This parameter can be a value of @ref SPI_Slave_Select_management */

    uint32_t NSSPMode;            /*!< Specifies whether the NSSP signal is enabled or not .
                                     This parameter can be a value of @ref SPI_NSSP_Mode
                                     This mode is activated by the NSSP bit in the SPIx_CR2 register and
                                     it takes effect only if the SPI interface is configured as Motorola SPI
                                     master (FRF=0) with capture on the first edge (SPIx_CR1 CPHA = 0,
                                     CPOL setting is ignored).. */
#endif
} SPI_InitTypeDef;

/**
  * @brief  HAL SPI State structure definition
  */
typedef enum
{
    HAL_SPI_STATE_RESET      = 0x00U,    /*!< Peripheral not Initialized                         */
    HAL_SPI_STATE_READY      = 0x01U,    /*!< Peripheral Initialized and ready for use           */
    HAL_SPI_STATE_BUSY       = 0x02U,    /*!< an internal process is ongoing                     */
    HAL_SPI_STATE_BUSY_TX    = 0x03U,    /*!< Data Transmission process is ongoing               */
    HAL_SPI_STATE_BUSY_RX    = 0x04U,    /*!< Data Reception process is ongoing                  */
    HAL_SPI_STATE_BUSY_TX_RX = 0x05U,    /*!< Data Transmission and Reception process is ongoing */
    HAL_SPI_STATE_ERROR      = 0x06U,    /*!< SPI error state                                    */
    HAL_SPI_STATE_ABORT      = 0x07U     /*!< SPI abort is ongoing                               */
} HAL_SPI_StateTypeDef;

/**
  * @brief  SPI handle Structure definition
  */
typedef struct __SPI_HandleTypeDef
{
    SPI_TypeDef                *Instance;      /*!< SPI registers base address               */

    SPI_InitTypeDef            Init;           /*!< SPI communication parameters             */

    uint8_t                    *pTxBuffPtr;    /*!< Pointer to SPI Tx transfer Buffer        */

    uint16_t                   TxXferSize;     /*!< SPI Tx Transfer size                     */

    __IO uint16_t              TxXferCount;    /*!< SPI Tx Transfer Counter                  */

    uint8_t                    *pRxBuffPtr;    /*!< Pointer to SPI Rx transfer Buffer        */

    uint16_t                   RxXferSize;     /*!< SPI Rx Transfer size                     */

    __IO uint16_t              RxXferCount;    /*!< SPI Rx Transfer Counter                  */

    uint32_t                   CRCSize;        /*!< SPI CRC size used for the transfer       */

    void (*RxISR)(struct __SPI_HandleTypeDef *hspi);   /*!< function pointer on Rx ISR       */

    void (*TxISR)(struct __SPI_HandleTypeDef *hspi);   /*!< function pointer on Tx ISR       */

    DMA_HandleTypeDef          *hdmatx;        /*!< SPI Tx DMA Handle parameters             */

    DMA_HandleTypeDef          *hdmarx;        /*!< SPI Rx DMA Handle parameters             */

    uint8_t                    core;           /*!< SPI Clock core source                        */

    HAL_LockTypeDef            Lock;           /*!< Locking object                           */

    __IO HAL_SPI_StateTypeDef  State;          /*!< SPI communication state                  */

    __IO uint32_t              ErrorCode;      /*!< SPI Error code                           */

} SPI_HandleTypeDef;

/* Receive or transmit request structure */
typedef struct
{
    uint8_t *pData;
    uint16_t Size;
    uint8_t IsRead;
} SPI_RtxRequestTypeDef;


/**
  * @} SPI_Exported_Types
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup SPI_Exported_Constants SPI Exported Constants
  * @{
  */

/** @defgroup SPI_Error_Code SPI Error Code
  * @{
  */
#define HAL_SPI_ERROR_NONE              (0x00000000U)   /*!< No error                               */
#define HAL_SPI_ERROR_MODF              (0x00000001U)   /*!< MODF error                             */
#define HAL_SPI_ERROR_CRC               (0x00000002U)   /*!< CRC error                              */
#define HAL_SPI_ERROR_OVR               (0x00000004U)   /*!< OVR error                              */
#define HAL_SPI_ERROR_FRE               (0x00000008U)   /*!< FRE error                              */
#define HAL_SPI_ERROR_DMA               (0x00000010U)   /*!< DMA transfer error                     */
#define HAL_SPI_ERROR_FLAG              (0x00000020U)   /*!< Error on RXNE/TXE/BSY/FTLVL/FRLVL Flag */
#define HAL_SPI_ERROR_ABORT             (0x00000040U)   /*!< Error during SPI Abort procedure       */
#define HAL_SPI_ERROR_UDR               (0x00000080U)   /*!< Underrun error       */
#define HAL_SPI_ERROR_EOC               (0x00000100U)   /*!< Endof dma chain       */
#define HAL_SPI_ERROR_PINT              (0x00000200U)   /*!< Peripheral Trailing Byte       */
#define HAL_SPI_ERROR_RXTO              (0x00000400U)   /*!< Receiver Time-out       */
#define HAL_SPI_ERROR_BCE               (0x00000800U)   /*!< Bit Count Error       */



/**
  * @} SPI_Error_Code
  */

/** @defgroup SPI_Mode SPI Mode
  * @{
  */
#define SPI_MODE_SLAVE                  (SPI_TOP_CTRL_SFRMDIR | SPI_TOP_CTRL_SCLKDIR)
#define SPI_MODE_MASTER                 (0x00000000U)
/**
  * @} SPI_Mode
  */

/** @defgroup SPI_Direction SPI Direction Mode
  * @{
  */
#define SPI_DIRECTION_2LINES            (0x00000003U)
#define SPI_DIRECTION_2LINES_RXONLY     (0x00000002U)   //TODO: hw doesn't support 4-wire spi
#define SPI_DIRECTION_1LINE             (0x00000001U)
/**
  * @} SPI_Direction
  */

/** @defgroup SPI_Data_Size SPI Data Size
  * @{
  */
#define SPI_DATASIZE_4BIT               (MAKE_REG_VAL(3, SPI_TOP_CTRL_DSS_Msk, SPI_TOP_CTRL_DSS_Pos))
#define SPI_DATASIZE_5BIT               (MAKE_REG_VAL(4, SPI_TOP_CTRL_DSS_Msk, SPI_TOP_CTRL_DSS_Pos))
#define SPI_DATASIZE_6BIT               (MAKE_REG_VAL(5, SPI_TOP_CTRL_DSS_Msk, SPI_TOP_CTRL_DSS_Pos))
#define SPI_DATASIZE_7BIT               (MAKE_REG_VAL(6, SPI_TOP_CTRL_DSS_Msk, SPI_TOP_CTRL_DSS_Pos))
#define SPI_DATASIZE_8BIT               (MAKE_REG_VAL(7, SPI_TOP_CTRL_DSS_Msk, SPI_TOP_CTRL_DSS_Pos))
#define SPI_DATASIZE_9BIT               (MAKE_REG_VAL(8, SPI_TOP_CTRL_DSS_Msk, SPI_TOP_CTRL_DSS_Pos))
#define SPI_DATASIZE_10BIT              (MAKE_REG_VAL(9, SPI_TOP_CTRL_DSS_Msk, SPI_TOP_CTRL_DSS_Pos))
#define SPI_DATASIZE_11BIT              (MAKE_REG_VAL(10, SPI_TOP_CTRL_DSS_Msk, SPI_TOP_CTRL_DSS_Pos))
#define SPI_DATASIZE_12BIT              (MAKE_REG_VAL(11, SPI_TOP_CTRL_DSS_Msk, SPI_TOP_CTRL_DSS_Pos))
#define SPI_DATASIZE_13BIT              (MAKE_REG_VAL(12, SPI_TOP_CTRL_DSS_Msk, SPI_TOP_CTRL_DSS_Pos))
#define SPI_DATASIZE_14BIT              (MAKE_REG_VAL(13, SPI_TOP_CTRL_DSS_Msk, SPI_TOP_CTRL_DSS_Pos))
#define SPI_DATASIZE_15BIT              (MAKE_REG_VAL(14, SPI_TOP_CTRL_DSS_Msk, SPI_TOP_CTRL_DSS_Pos))
#define SPI_DATASIZE_16BIT              (MAKE_REG_VAL(15, SPI_TOP_CTRL_DSS_Msk, SPI_TOP_CTRL_DSS_Pos))
/**
  * @} SPI_Data_Size
  */

/** @defgroup SPI_Clock_Polarity SPI Clock Polarity
  * @{
  */
#define SPI_POLARITY_LOW                (0x00000000U)
#define SPI_POLARITY_HIGH               SPI_TOP_CTRL_SPO
/**
  * @} SPI_Clock_Polarity
  */

/** @defgroup SPI_Clock_Phase SPI Clock Phase
  * @{
  */
#define SPI_PHASE_1EDGE                 (0x00000000U)
#define SPI_PHASE_2EDGE                 SPI_TOP_CTRL_SPH
/**
  * @} SPI_Clock_Phase
  */

/** @defgroup SPI_SFRM_POL SPI synchronous signal Polarity
  * @{
  */
#define SPI_SFRMPOL_LOW                  (0x00000000U)
#define SPI_SFRMPOL_HIGH                 SPI_TOP_CTRL_IFS
/**
  * @} SPI_SFRM_POL
  */

#ifdef SPI_FRM_HDR_DATA_HDR_DATA
#define SPI_BAUDRATE_PRESCALER_MAX       (SPI_TOP_CTRL_CLK_DIV_Msk >> SPI_TOP_CTRL_CLK_DIV_Pos)
#else
#define SPI_BAUDRATE_PRESCALER_MAX       (SPI_CLK_CTRL_CLK_DIV_Msk >> SPI_CLK_CTRL_CLK_DIV_Pos)
#endif

/** @defgroup SPI_Interrupt_definition SPI Interrupt Definition
  * @{
  */
#define SPI_IT_TXE                      SPI_INTE_TIE
#define SPI_IT_RXNE                     SPI_INTE_RIE
#define SPI_IT_ERR                      (SPI_INTE_EBCEI | SPI_INTE_TINTE | SPI_INTE_PINTE)
#define SPI_MSK_IT_ERR                  (SPI_INTE_TIM | SPI_INTE_RIM) //This is irq MASK NOT ENABLE, and SPI_IT_xxx above are ENABLE
/**
  * @} SPI_Interrupt_definition
  */

/** @defgroup SPI_Flags_definition SPI Flags Definition
  * @{
  */
#define SPI_FLAG_RXNE                   SPI_STATUS_RNE   /* SPI status flag: Rx buffer not empty flag       */
#define SPI_FLAG_TXE                    SPI_STATUS_TNF    /* SPI status flag: Tx buffer empty flag           */
#define SPI_FLAG_BSY                    SPI_STATUS_BSY    /* SPI status flag: Busy flag                      */
//#define SPI_FLAG_CRCERR                 SPI_SR_CRCERR /* SPI Error flag: CRC error flag                  */
//#define SPI_FLAG_MODF                   SPI_SR_MODF   /* SPI Error flag: Mode fault flag                 */
#define SPI_FLAG_OVR                    SPI_STATUS_ROR    /* SPI Error flag: Overrun flag                    */
#define SPI_FLAG_UDR                    SPI_STATUS_TUR    /* SPI Error flag: Underrun flag                    */
//#define SPI_FLAG_FRE                    SPI_STATUS_BCE    /* SPI Error flag: Frame format error flag,bit count error */
#define SPI_FLAG_FTLVL                  (SPI_STATUS_TFL_Msk | SPI_STATUS_TNF_Msk)  /* SPI fifo transmission level                     */
#define SPI_FLAG_FRLVL                  (SPI_STATUS_RFL_Msk | SPI_STATUS_RNE_Msk)  /* SPI fifo reception level                        */
#define SPI_FLAG_EXT                   (SPI_STATUS_EOC| SPI_STATUS_PINT|SPI_STATUS_TINT|SPI_STATUS_BCE)    /* SPI Extend Error flag:                     */

/**
  * @} SPI_Flags_definition
  */


/** @defgroup SPI_transmission_fifo_status_level SPI Transmission FIFO Status Level
  * @{
  */
#define SPI_FTLVL_EMPTY                 ((0 << SPI_STATUS_TFL_Pos)|SPI_STATUS_TNF)
//#define SPI_FTLVL_QUARTER_FULL          (0x00000800U)
//#define SPI_FTLVL_HALF_FULL             (0x00001000U)
#define SPI_FTLVL_FULL                  ((0 << SPI_STATUS_TFL_Pos))

/**
  * @} SPI_transmission_fifo_status_level
  */

/** @defgroup SPI_reception_fifo_status_level SPI Reception FIFO Status Level
  * @{
  */
#define SPI_FRLVL_EMPTY                 (SPI_STATUS_RFL_Msk)
#define SPI_FRLVL_FULL                  ((SPI_STATUS_RFL_Msk) | SPI_STATUS_RNE)
/**
  * @} SPI_reception_fifo_status_level
  */

/** @defgroup SPI_Frame_Format SPI data format
  * @{
  */
#define SPI_FRAME_FORMAT_SPI            (0x00000000U << SPI_TOP_CTRL_FRF_Pos)
#define SPI_FRAME_FORMAT_SSP            (0x00000001U << SPI_TOP_CTRL_FRF_Pos)
#define SPI_FRAME_FORMAT_NM             (0x00000002U << SPI_TOP_CTRL_FRF_Pos)
#define SPI_FRAME_FORMAT_FRM            (0x00000003U << SPI_TOP_CTRL_FRF_Pos)

/**
  * @} SPI_Frame_Format
  */


/** @defgroup SPI_CRC_Calculation SPI CRC Calculation
  * @{
  */
#define SPI_CRCCALCULATION_DISABLE      (0x00000000U)
#define SPI_CRCCALCULATION_ENABLE       SPI_CR1_CRCEN
/**
  * @} SPI_CRC_Calculation
  */

/** @defgroup SPI_CRC_length SPI CRC Length
  * @{
  * This parameter can be one of the following values:
  *     SPI_CRC_LENGTH_DATASIZE: aligned with the data size
  *     SPI_CRC_LENGTH_8BIT    : CRC 8bit
  *     SPI_CRC_LENGTH_16BIT   : CRC 16bit
  */
#define SPI_CRC_LENGTH_DATASIZE         (0x00000000U)
#define SPI_CRC_LENGTH_8BIT             (0x00000001U)
#define SPI_CRC_LENGTH_16BIT            (0x00000002U)
/**
  * @} SPI_CRC_length
  */

/** @defgroup SPI_FIFO_reception_threshold SPI FIFO Reception Threshold
  * @{
  * This parameter can be one of the following values:
  *     SPI_RXFIFO_THRESHOLD or SPI_RXFIFO_THRESHOLD_QF :
  *          RXNE event is generated if the FIFO
  *          level is greater or equal to 1/2(16-bits).
  *     SPI_RXFIFO_THRESHOLD_HF: RXNE event is generated if the FIFO
  *          level is greater or equal to 1/4(8 bits). */
#define SPI_RXFIFO_THRESHOLD            SPI_CR2_FRXTH
#define SPI_RXFIFO_THRESHOLD_QF         SPI_CR2_FRXTH
#define SPI_RXFIFO_THRESHOLD_HF         (0x00000000U)
/**
  * @}SPI_FIFO_reception_threshold
  */

/**
  * @} SPI_Exported_Constants
  */


/* Exported macros -----------------------------------------------------------*/
/** @defgroup SPI_Exported_Macros SPI Exported Macros
  * @{
  */

/** @brief  Reset SPI handle state.
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_SPI_STATE_RESET)


/** @brief  Return true if the specified SPI interrupts is enabled.
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  \__INTERRUPT__ specifies the interrupt source to enable.
  *         This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval None
  */
#define __HAL_SPI_IS_ENABLE_IT(__HANDLE__, __INTERRUPT__)   HAL_IS_BIT_SET((__HANDLE__)->Instance->INTE, (__INTERRUPT__))


/** @brief  Enable the specified SPI interrupts.
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  \__INTERRUPT__ specifies the interrupt source to enable.
  *         This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval None
  */
#define __HAL_SPI_ENABLE_IT(__HANDLE__, __INTERRUPT__)   do{__IO uint32_t tmpreg_ite = (__HANDLE__)->Instance->INTE;\
                                                            SET_BIT(tmpreg_ite, (__INTERRUPT__));\
                                                          if(HAL_IS_BIT_SET((__INTERRUPT__),SPI_IT_ERR)){\
                                                            CLEAR_BIT(tmpreg_ite, SPI_MSK_IT_ERR);}\
                                                            (__HANDLE__)->Instance->INTE = tmpreg_ite;\
                                                            }while(0)

/** @brief  Disable the specified SPI interrupts.
  * @param  \__HANDLE__ specifies the SPI handle.
  *         This parameter can be SPIx where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  \__INTERRUPT__ specifies the interrupt source to disable.
  *         This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval None
  */
#define __HAL_SPI_DISABLE_IT(__HANDLE__, __INTERRUPT__)  do{__IO uint32_t tmpreg_ite = (__HANDLE__)->Instance->INTE;\
                                                            CLEAR_BIT(tmpreg_ite, (__INTERRUPT__));\
                                                          if(HAL_IS_BIT_SET((__INTERRUPT__),SPI_IT_ERR)){\
                                                            SET_BIT(tmpreg_ite, SPI_MSK_IT_ERR);}\
                                                            (__HANDLE__)->Instance->INTE = tmpreg_ite;\
                                                            }while(0)

/** @brief  Check whether the specified SPI interrupt source is enabled or not.
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  \__INTERRUPT__ specifies the SPI interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
#define __HAL_SPI_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->Instance->CR1 & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/** @brief  Check whether the specified SPI flag is set or not.
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  \__FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_FLAG_RXNE: Receive buffer not empty flag
  *            @arg SPI_FLAG_TXE: Transmit buffer empty flag
  *            @arg SPI_FLAG_CRCERR: CRC error flag
  *            @arg SPI_FLAG_MODF: Mode fault flag
  *            @arg SPI_FLAG_OVR: Overrun flag
  *            @arg SPI_FLAG_BSY: Busy flag
  *            @arg SPI_FLAG_FRE: Frame format error flag
  *            @arg SPI_FLAG_FTLVL: SPI fifo transmission level
  *            @arg SPI_FLAG_FRLVL: SPI fifo reception level
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_SPI_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->STATUS) & (__FLAG__)) == (__FLAG__))


/** @brief  Clear the SPI OVR pending flag.
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_CLEAR_OVRFLAG(__HANDLE__)        \
  do{                                              \
    __IO uint32_t tmpreg_ovr = 0x00U;              \
    tmpreg_ovr = (__HANDLE__)->Instance->DATA;     \
    (__HANDLE__)->Instance->STATUS = SPI_FLAG_OVR; \
    UNUSED(tmpreg_ovr);                            \
  } while(0U)

/** @brief  Clear the SPI FRE pending flag.
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_CLEAR_FREFLAG(__HANDLE__)        \
  do{                                              \
  __IO uint32_t tmpreg_fre = 0x00U;                \
  UNUSED(tmpreg_fre);                              \
  }while(0U)



/** @brief  Clear the SPI UDR pending flag.
* @param  \__HANDLE__ specifies the SPI Handle.
*         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
* @retval None
*/
#define __HAL_SPI_CLEAR_UDRFLAG(__HANDLE__)        \
do{                                              \
__IO uint32_t tmpreg_ovr = 0x00U;              \
(__HANDLE__)->Instance->STATUS = SPI_FLAG_UDR; \
UNUSED(tmpreg_ovr);                            \
} while(0U)


/** @brief  Clear the SPI extend error flag.
* @param  \__HANDLE__ specifies the SPI Handle.
*         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
* @retval None
*/
#define __HAL_SPI_CLEAR_EXT_ERR_FLAG(__HANDLE__)        \
do{                                              \
__IO uint32_t tmpreg_ovr = 0x00U;              \
(__HANDLE__)->Instance->STATUS = SPI_FLAG_EXT; \
UNUSED(tmpreg_ovr);                            \
} while(0U)


/** @brief  Enable the SPI peripheral.
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_ENABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->TOP_CTRL, SPI_TOP_CTRL_SSE)

/** @brief  Disable the SPI peripheral.
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_DISABLE(__HANDLE__) CLEAR_BIT((__HANDLE__)->Instance->TOP_CTRL, SPI_TOP_CTRL_SSE)

/** @brief  Take SPI CS (to low).
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_TAKE_CS(__HANDLE__)      SET_BIT((__HANDLE__)->Instance->TOP_CTRL, SPI_TOP_CTRL_HOLD_FRAME_LOW)

/** @brief  Release SPI CS (to high).
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_RELEASE_CS(__HANDLE__)       CLEAR_BIT((__HANDLE__)->Instance->TOP_CTRL, SPI_TOP_CTRL_HOLD_FRAME_LOW)

#ifdef SPI_FRM_HDR_DATA_HDR_DATA
/** @brief  Frame Mode(tx after header).
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_FRM_MODE_TX(__HANDLE__)       SET_BIT((__HANDLE__)->Instance->TOP_CTRL2, SPI_TOP_CTRL2_FRM_MODE)

/** @brief  Frame Mode(rx after header).
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_FRM_MODE_RX(__HANDLE__)       CLEAR_BIT((__HANDLE__)->Instance->TOP_CTRL2, SPI_TOP_CTRL2_FRM_MODE)

/*header len in bit 0-32, size low 8bit.*/
#define __HAL_SPI_FRM_HDR_LEN(SIZE)                     ((SIZE & 0xFF) - 1)
/*data size in byte or halfword, high 24bit.*/
#define __HAL_SPI_FRM_DAT_SIZE(SIZE)                    (SIZE >> 8)

#define __HAL_SPI_GET_DATA_SIZE(__HANDLE__)             ((READ_BIT((__HANDLE__)->Instance->TOP_CTRL, SPI_TOP_CTRL_DSS) >> SPI_TOP_CTRL_DSS_Pos) + 1)

#define __HAL_SPI_FRM_DAT_SIZE_BIT(__HANDLE__, SIZE)    (__HAL_SPI_FRM_DAT_SIZE(SIZE) * __HAL_SPI_GET_DATA_SIZE(__HANDLE__))

/** @brief  Frame Mode Write Header Data.
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_FRM_HDR_DATA_WRITE(__HANDLE__, PDATA)     ((__HANDLE__)->Instance->FRM_HDR_DATA = *((uint32_t *)PDATA))

/** @brief  Frame Mode Set Header Length.
 * @param  \__HANDLE__ specifies the SPI Handle.
 *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
 * @retval None
 */
#define __HAL_SPI_FRM_HDR_LEN_SET(__HANDLE__, SIZE)         MODIFY_REG((__HANDLE__)->Instance->TOP_CTRL2, SPI_TOP_CTRL2_FRM_HDR_LENGTH, (__HAL_SPI_FRM_HDR_LEN(SIZE) << SPI_TOP_CTRL2_FRM_HDR_LENGTH_Pos))

/** @brief  Frame Mode Set Data Size.
 * @param  \__HANDLE__ specifies the SPI Handle.
 *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
 * @retval None
 */
#define __HAL_SPI_FRM_DAT_SIZE_SET(__HANDLE__, SIZE)        MODIFY_REG((__HANDLE__)->Instance->TOP_CTRL2, SPI_TOP_CTRL2_SSPRWOTCCM, ((__HAL_SPI_FRM_DAT_SIZE_BIT(__HANDLE__, SIZE)) << SPI_TOP_CTRL2_SSPRWOTCCM_Pos))

/** @brief  Frame Mode Set RWOT CYCLE.
 * @param  \__HANDLE__ specifies the SPI Handle.
 *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
 * @retval None
 */
#define __HAL_SPI_FRM_RWOT_CYC_SET(__HANDLE__)              SET_BIT(hspi->Instance->TOP_CTRL2, SPI_TOP_CTRL2_SET_RWOT_CYCLE);

/** @brief  Frame Mode Start.
 * @param  \__HANDLE__ specifies the SPI Handle.
 *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
 * @retval None
 */
#define __HAL_SPI_FRM_MODE_START(__HANDLE__)                SET_BIT(hspi->Instance->TOP_CTRL2, SPI_TOP_CTRL2_FRM_START)

/** @brief  Frame Mode Stop.
 * @param  \__HANDLE__ specifies the SPI Handle.
 *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
 * @retval None
 */
#define __HAL_SPI_FRM_MODE_STOP(__HANDLE__)                CLEAR_BIT(hspi->Instance->TOP_CTRL2, SPI_TOP_CTRL2_FRM_START)
#endif /*SPI_FRM_HDR_DATA_HDR_DATA*/

#if (USE_SPI_CRC != 0)
/** @brief  Clear the SPI CRCERR pending flag.
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_CLEAR_CRCERRFLAG(__HANDLE__) ((__HANDLE__)->Instance->SR = (uint16_t)(~SPI_FLAG_CRCERR))
#endif /* USE_SPI_CRC */

/** @brief  Clear the SPI MODF pending flag.
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_CLEAR_MODFFLAG(__HANDLE__)             \
  do{                                                    \
    __IO uint32_t tmpreg_modf = 0x00U;                   \
    tmpreg_modf = (__HANDLE__)->Instance->SR;            \
    CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_SPE); \
    UNUSED(tmpreg_modf);                                 \
  } while(0U)

/**
  * @} SPI_Exported_Macros
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup SPI_Private_Macros SPI Private Macros
  * @{
  */


/** @brief  Set the SPI transmit-only mode.
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#ifdef SPI_FRM_HDR_DATA_HDR_DATA
#define SPI_1LINE_TX(__HANDLE__)  CLEAR_BIT((__HANDLE__)->Instance->TOP_CTRL, SPI_TOP_CTRL_TXD_OEN)
#else
#define SPI_1LINE_TX(__HANDLE__)  CLEAR_BIT((__HANDLE__)->Instance->TRIWIRE_CTRL, SPI_TRIWIRE_CTRL_TXD_OEN)
#endif


/** @brief  Set the SPI receive-only mode.
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#ifdef SPI_FRM_HDR_DATA_HDR_DATA
#define SPI_1LINE_RX(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->TOP_CTRL, SPI_TOP_CTRL_TXD_OEN)
#else
#define SPI_1LINE_RX(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->TRIWIRE_CTRL, SPI_TRIWIRE_CTRL_TXD_OEN)
#endif

/** @brief  Reset the CRC calculation of the SPI.
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define SPI_RESET_CRC(__HANDLE__) do{CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_CRCEN);\
                                       SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_CRCEN);}while(0U)
/** @brief  Set the ssp clock Cycles when SSP works in master and RWOT mode.
  * @param  \__HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  \NUM specifies the ssp clock Cycles.
  * @retval None
  */
#ifdef SF32LB57X
/* On SF32LB57X, SSPRWOTCCM field is in TOP_CTRL2[31:16] */
#define SPI_RWOT_CCM(__HANDLE__, NUM)  MODIFY_REG((__HANDLE__)->Instance->TOP_CTRL2, SPI_TOP_CTRL2_SSPRWOTCCM, MAKE_REG_VAL2((NUM), SPI_TOP_CTRL2_SSPRWOTCCM))
#else
#define SPI_RWOT_CCM(__HANDLE__, NUM)  ((__HANDLE__)->Instance->RWOT_CCM = (uint32_t)(NUM))
#endif

/** @brief  Set the SPI RWOT Receive without transmit mode.
 * @param  \__HANDLE__ specifies the SPI Handle.
            This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
 * @retval None
 */
#ifdef SF32LB57X
#define  SPI_SET_RWOT_RECEIVE_WITHOUT_TRANSMIT_MODE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->TOP_CTRL2, SPI_TOP_CTRL2_RWOT)
/** @brief  Set the SPI RWOT Transmit/receive mode.
* @param  \__HANDLE__ specifies the SPI Handle.
*         0 = Transmit/receive mode; 1 = Receive without transmit mode.
* @retval None
*/
#define SPI_SET_RWOT_TRANSMIT_RECEIVE_MODE(__HANDLE__)  CLEAR_BIT((__HANDLE__)->Instance->TOP_CTRL2, SPI_TOP_CTRL2_RWOT)
#else
#define  SPI_SET_RWOT_RECEIVE_WITHOUT_TRANSMIT_MODE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->RWOT_CTRL, SPI_RWOT_CTRL_RWOT)
/** @brief  Set the SPI RWOT Transmit/receive mode.
* @param  \__HANDLE__ specifies the SPI Handle.
*         0 = Transmit/receive mode; 1 = Receive without transmit mode.
* @retval None
*/
#define SPI_SET_RWOT_TRANSMIT_RECEIVE_MODE(__HANDLE__)  CLEAR_BIT((__HANDLE__)->Instance->RWOT_CTRL, SPI_RWOT_CTRL_RWOT)
#endif

/** @brief  Enable RWOT Cycle Counter Mode
* @param  \__HANDLE__ specifies the SPI Handle.
*         0 = disable; 0 = Receive without transmit mode.
* @retval None
*/
#ifdef SF32LB57X
#define SPI_RWOT_CYCEL_ENABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->TOP_CTRL2, SPI_TOP_CTRL2_CYCLE_RWOT_EN)
#else
#define SPI_RWOT_CYCEL_ENABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->RWOT_CTRL, SPI_RWOT_CTRL_CYCLE_RWOT_EN)
#endif

/** @brief  Disable RWOT Cycle Counter Mode
* @param  \__HANDLE__ specifies the SPI Handle.
*         0 = disable; 0 = Receive without transmit mode.
* @retval None
*/
#ifdef SF32LB57X
#define SPI_RWOT_CYCEL_DISABLE(__HANDLE__)  CLEAR_BIT((__HANDLE__)->Instance->TOP_CTRL2, SPI_TOP_CTRL2_CYCLE_RWOT_EN)
#else
#define SPI_RWOT_CYCEL_DISABLE(__HANDLE__)  CLEAR_BIT((__HANDLE__)->Instance->RWOT_CTRL, SPI_RWOT_CTRL_CYCLE_RWOT_EN)
#endif

/** @brief  Set RWOT Cycle
* @param  \__HANDLE__ specifies the SPI Handle.
*         0 = disable; 0 = Receive without transmit mode.
* @retval None
*/
#ifdef SF32LB57X
#define SPI_RWOT_SET_CYCEL(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->TOP_CTRL2, SPI_TOP_CTRL2_SET_RWOT_CYCLE)
#else
#define SPI_RWOT_SET_CYCEL(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->RWOT_CTRL, SPI_RWOT_CTRL_SET_RWOT_CYCLE)
#endif

/** @brief  Clear RWOT Cycle
* @param  \__HANDLE__ specifies the SPI Handle.
*         0 = disable; 0 = Receive without transmit mode.
* @retval None
*/
#ifdef SF32LB57X
#define SPI_RWOT_CLEAR_CYCEL(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->TOP_CTRL2, SPI_TOP_CTRL2_CLR_RWOT_CYCLE)
#else
#define SPI_RWOT_CLEAR_CYCEL(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->RWOT_CTRL, SPI_RWOT_CTRL_CLR_RWOT_CYCLE)
#endif


#define IS_SPI_MODE(MODE) (((MODE) == SPI_MODE_SLAVE) || \
                           ((MODE) == SPI_MODE_MASTER))

#define IS_SPI_DIRECTION(MODE) (((MODE) == SPI_DIRECTION_2LINES)        || \
                                ((MODE) == SPI_DIRECTION_2LINES_RXONLY) || \
                                ((MODE) == SPI_DIRECTION_1LINE))

#define IS_SPI_DIRECTION_2LINES(MODE) ((MODE) == SPI_DIRECTION_2LINES)

#define IS_SPI_DIRECTION_2LINES_OR_1LINE(MODE) (((MODE) == SPI_DIRECTION_2LINES) || \
                                                ((MODE) == SPI_DIRECTION_1LINE))

#define IS_SPI_DATASIZE(DATASIZE) (((DATASIZE) == SPI_DATASIZE_16BIT) || \
                                   ((DATASIZE) == SPI_DATASIZE_15BIT) || \
                                   ((DATASIZE) == SPI_DATASIZE_14BIT) || \
                                   ((DATASIZE) == SPI_DATASIZE_13BIT) || \
                                   ((DATASIZE) == SPI_DATASIZE_12BIT) || \
                                   ((DATASIZE) == SPI_DATASIZE_11BIT) || \
                                   ((DATASIZE) == SPI_DATASIZE_10BIT) || \
                                   ((DATASIZE) == SPI_DATASIZE_9BIT)  || \
                                   ((DATASIZE) == SPI_DATASIZE_8BIT)  || \
                                   ((DATASIZE) == SPI_DATASIZE_7BIT)  || \
                                   ((DATASIZE) == SPI_DATASIZE_6BIT)  || \
                                   ((DATASIZE) == SPI_DATASIZE_5BIT)  || \
                                   ((DATASIZE) == SPI_DATASIZE_4BIT))

#define IS_SPI_CPOL(CPOL) (((CPOL) == SPI_POLARITY_LOW) || \
                           ((CPOL) == SPI_POLARITY_HIGH))

#define IS_SPI_CPHA(CPHA) (((CPHA) == SPI_PHASE_1EDGE) || \
                           ((CPHA) == SPI_PHASE_2EDGE))

#define IS_SPI_FRAME_FORMAT(MODE) (((MODE) == SPI_FRAME_FORMAT_SPI) || \
                             ((MODE) == SPI_FRAME_FORMAT_SSP) || \
                             ((MODE) == SPI_FRAME_FORMAT_NM) || \
                             ((MODE) == SPI_FRAME_FORMAT_FRM))

#define IS_SPI_MASTER(MODE) (((MODE) == SPI_MODE_MASTER))

#define IS_SPI_SFRMPOL(SFRMPOL) (((SFRMPOL) == SPI_SFRMPOL_LOW) || \
                                 ((SFRMPOL) == SPI_SFRMPOL_HIGH))

#define IS_SPI_CRC_CALCULATION(CALCULATION) (((CALCULATION) == SPI_CRCCALCULATION_DISABLE) || \
                                             ((CALCULATION) == SPI_CRCCALCULATION_ENABLE))

#define IS_SPI_CRC_LENGTH(LENGTH) (((LENGTH) == SPI_CRC_LENGTH_DATASIZE) ||\
                                   ((LENGTH) == SPI_CRC_LENGTH_8BIT)  ||   \
                                   ((LENGTH) == SPI_CRC_LENGTH_16BIT))

#define IS_SPI_CRC_POLYNOMIAL(POLYNOMIAL) (((POLYNOMIAL) >= 0x1U) && ((POLYNOMIAL) <= 0xFFFFU) && (((POLYNOMIAL)&0x1U) != 0U))

#define IS_SPI_DMA_HANDLE(HANDLE) ((HANDLE) != NULL)

#define IS_SPI_16BIT_ALIGNED_ADDRESS(DATA) (((uint32_t)(DATA) % 2U) == 0U)


#if defined(SF32LB52X) || defined(SF32LB57X)
#define IS_SPI_ALL_INSTANCE(__INSTANCE__)      (((__INSTANCE__) == SPI1) || \
                                                ((__INSTANCE__) == SPI2))
#else
#define IS_SPI_ALL_INSTANCE(__INSTANCE__)      (((__INSTANCE__) == SPI1) || \
                                                ((__INSTANCE__) == SPI2) || \
                                                ((__INSTANCE__) == SPI3) || \
                                                ((__INSTANCE__) == SPI4))
#endif

#define IS_SPI_BAUDRATE_PRESCALER(PRESCALER) (((PRESCALER) > 0x0U) && ((PRESCALER) <= SPI_BAUDRATE_PRESCALER_MAX))

/**
  * @} SPI_Private_Macros
  */


/* Exported functions --------------------------------------------------------*/
/** @defgroup SPI_Exported_Functions SPI Exported Functions
  * @{
  */

/** @defgroup SPI_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the SPIx peripheral:

      (+) User must implement HAL_SPI_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO, DMA, IT and NVIC ).

      (+) Call the function HAL_SPI_Init() to configure the selected device with
          the selected configuration:
        (++) Mode
        (++) Direction
        (++) Data Size
        (++) Clock Polarity and Phase
        (++) BaudRate Prescaler
        (++) FrameFormat
        (++) CRC Calculation
        (++) CRC Polynomial if CRC enabled
        (++) CRC Length, used only with Data8 and Data16
        (++) FIFO reception threshold

      (+) Call the function HAL_SPI_DeInit() to restore the default configuration
          of the selected SPIx peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the SPI according to the specified parameters
  *         in the SPI_InitTypeDef and initialize the associated handle.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);

/**
  * @brief  De-Initialize the SPI peripheral.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *hspi);

/**
  * @brief  Initialize the SPI MSP.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);

/**
  * @brief  De-Initialize the SPI MSP.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi);
/**
  * @} SPI_Exported_Functions_Group1
  */

/** @defgroup SPI_Exported_Functions_Group2 IO operation functions
 *  @brief   Data transfers functions
 *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
 ===============================================================================
 [..]
    This subsection provides a set of functions allowing to manage the SPI
    data transfers.

    [..] The SPI supports master and slave mode :

    (#) There are two modes of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The HAL status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode: The communication is performed using Interrupts
            or DMA, These APIs return the HAL status.
            The end of the data processing will be indicated through the
            dedicated SPI IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.
            The HAL_SPI_TxCpltCallback(), HAL_SPI_RxCpltCallback() and HAL_SPI_TxRxCpltCallback() user callbacks
            will be executed respectively at the end of the transmit or Receive process
            The HAL_SPI_ErrorCallback()user callback will be executed when a communication error is detected

    (#) APIs provided for these 2 transfer modes (Blocking mode or Non blocking mode using either Interrupt or DMA)
        exist for 1Line (simplex) and 2Lines (full duplex) modes.

@endverbatim
  * @{
  */

/**
  * @brief  Transmit an amount of data in blocking mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
/**
  * @brief Flush the RX fifo.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_FlushRxFifo(SPI_HandleTypeDef *hspi);

/**
  * @brief  Receive an amount of data in blocking mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be received
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
/**
  * @brief  Transmit and Receive an amount of data in blocking mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @param  Size amount of data to be sent and received
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
        uint32_t Timeout);

/**
  * @brief  Transmit an amount of data in non-blocking mode with Interrupt.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);

/**
  * @brief  Receive an amount of data in non-blocking mode with Interrupt.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);

/**
  * @brief  Transmit and Receive an amount of data in non-blocking mode with Interrupt.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @param  Size amount of data to be sent and received
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
        uint16_t Size);


/**
  * @brief  Transmit an amount of data in non-blocking mode with DMA.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);

/**
  * @brief  Receive an amount of data in non-blocking mode with DMA.
  * @note   In case of MASTER mode and SPI_DIRECTION_2LINES direction, hdmatx shall be defined.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @note   When the CRC feature is enabled the pData Length must be Size + 1.
  * @param  Size amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);

/**
  * @brief  Transmit and Receive an amount of data in non-blocking mode with DMA.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @note   When the CRC feature is enabled the pRxData Length must be Size + 1
  * @param  Size amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
        uint16_t Size);

#ifdef DMA_LINK_LIST_SUPPORT
/**
 * @brief  Get command buffer size for multiple transfer in DMA mode
 * @param  Requests list of multiple transfer requests
 * @param  Num transfer request number
 * @retval command buffer size
 */
uint32_t HAL_SPI_GetMultiRtxCmdBufSize(SPI_RtxRequestTypeDef *Requests, uint16_t Num);

/**
 * @brief  Prepare command buffer for multiple transfer in DMA mode,
 * the command update would be updated, which will be used by HAL_SPI_TransmitReceiveMultiple_DMA
 * @param[in]  hspi pointer to spi handle
 * @param[in]  Request list of multiple transfer requests
 * @param[in]  Num transfer request number
 * @param[in,out] CmdBuf Command buffer, the buffer would be updated by the function accor
 * @param[in]  BufSize Command buffer size, the size is calculated by HAL_SPI_GetMultiRtxCmdBufSize
 * @retval hal status
 */
HAL_StatusTypeDef HAL_SPI_PrepareMultiRtxCmdBuf(SPI_HandleTypeDef *hspi, SPI_RtxRequestTypeDef *Request, uint16_t Num, void *CmdBuf, uint32_t BufSize);

/**
 * @brief  Start multiple transfer according to command buffer
 * @param[in] hspi pointer to spi handle
 * @param[in] CmdBuf Command buffer, the buffer is prepared by HAL_SPI_PrepareMultiRtxCmdBuf
 * @retval hal status
 */
HAL_StatusTypeDef HAL_SPI_TransmitReceiveMultiple_DMA(SPI_HandleTypeDef *hspi, void *CmdBuf);
#endif /* DMA_LINK_LIST_SUPPORT */

/* Transfer Abort functions */
/**
  * @brief  Abort ongoing transfer (blocking mode).
  * @param  hspi SPI handle.
  * @note   This procedure could be used for aborting any ongoing transfer (Tx and Rx),
  *         started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable SPI Interrupts (depending of transfer direction)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval HAL status
*/
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *hspi);

/**
  * @brief  Abort ongoing transfer (Interrupt mode).
  * @param  hspi SPI handle.
  * @note   This procedure could be used for aborting any ongoing transfer (Tx and Rx),
  *         started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable SPI Interrupts (depending of transfer direction)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort_IT (in case of transfer in DMA mode)
  *           - Set handle State to READY
  *           - At abort completion, call user abort complete callback
  * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *         considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval HAL status
*/
HAL_StatusTypeDef HAL_SPI_Abort_IT(SPI_HandleTypeDef *hspi);

/**
  * @brief  Pause the DMA Transfer.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_DMAPause(SPI_HandleTypeDef *hspi);

/**
  * @brief  Resume the DMA Transfer.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_DMAResume(SPI_HandleTypeDef *hspi);

/**
  * @brief Stop the DMA Transfer.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *hspi);

/**
  * @brief  Handle SPI interrupt request.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval None
  */
void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);

/**
  * @brief Tx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);

/**
  * @brief Rx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);

/**
  * @brief Tx and Rx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);

/**
  * @brief Tx Half Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi);

/**
  * @brief Rx Half Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi);

/**
  * @brief Tx and Rx Half Transfer callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi);

/**
  * @brief SPI error callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

/**
  * @brief  SPI Abort Complete callback.
  * @param  hspi SPI handle.
  * @retval None
  */
void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi);
/**
  * @} SPI_Exported_Functions_Group2
  */

/** @defgroup SPI_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   SPI control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the SPI.
     (+) HAL_SPI_GetState() API can be helpful to check in run-time the state of the SPI peripheral
     (+) HAL_SPI_GetError() check in run-time Errors occurring during communication
@endverbatim
  * @{
  */

/**
  * @brief  Return the SPI handle state.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval SPI state
  */
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *hspi);

/**
  * @brief  Return the SPI error code.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval SPI error code in bitmap format
  */
uint32_t             HAL_SPI_GetError(SPI_HandleTypeDef *hspi);
/**
  * @} SPI_Exported_Functions_Group3
  */

/**
  * @} SPI_Exported_Functions
  */

/**
  * @} SPI
  */

/**
  * @} BF0_HAL_Driver
  */

#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_SPI_H */