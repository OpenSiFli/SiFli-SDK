/*
 * SPDX-FileCopyrightText: 2016 STMicroelectronics
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BF0_HAL_CAN_H
#define BF0_HAL_CAN_H


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup BF0_HAL_Driver
 * @{
 */

/** @addtogroup CAN
 * @{
 */

/** @defgroup CAN_RX_Software_FIFO_Depth CAN RX Software FIFO Depth
 *   56X has 7-slot hardware RBUF, 58X has 16-slot. Software FIFO matches.
 * @{
 */
#if defined(SF32LB58X)
#define CAN_RX_FIFO_DEPTH              16
#define CAN_SECONDARY_TX_FIFO_DEPTH    16
#else
#define CAN_RX_FIFO_DEPTH              7
#define CAN_SECONDARY_TX_FIFO_DEPTH    7
#endif
/**
 * @}
 */

#define CAN_MAX_STD_ID        (0x7FFU)        /*!< Max standard ID value */
#define CAN_MAX_EXT_ID        (0x1FFFFFFFU)   /*!< Max extended ID value */


/** @defgroup CAN_TIME_SEG1_RANGE CAN phase segment 1 range (including propagation segment)
 *
 * @{
 */
/**  minimum time quanta of phase segment 1 */
#define CAN_TIME_SEG1_MIN    (2)
/**  maximum time quanta of phase segment 1 */
#define CAN_TIME_SEG1_MAX    (16)
/**
 * @}
 */

/** @defgroup CAN_TIME_SEG2_RANGE CAN phase segment 2 range
 *
 * @{
 */
/**  minimum time quanta of phase segment 2 */
#define CAN_TIME_SEG2_MIN    (2)
/**  maximum time quanta of phase segment 2 */
#define CAN_TIME_SEG2_MAX    (8)
/**
 * @}
 */

/** @defgroup CAN_SYNC_JUMP_WIDTH_RANGE Sync Jump Width (SJW) range
 *
 * @{
 */
/**  minimum time quanta of phase segment 2 */
#define CAN_SYNC_JUMP_WIDTH_MIN    (1)
/**  maximum time quanta of phase segment 2 */
#define CAN_SYNC_JUMP_WIDTH_MAX    (4)
/**
 * @}
 */

/** @defgroup CAN_PRESCALER_RANGE prescaler range
 *
 * @{
 */
/**  minimum prescaler */
#define CAN_PRESCALER_MIN    (2)
/**  maximum prescaler */
#define CAN_PRESCALER_MAX    (GET_REG_VAL2(CAN_PRESC_S_PRESC_Msk, CAN_PRESC_S_PRESC) + 1)
/**
 * @}
 */


/**
 * @brief  CAN frame structures definition
 */
typedef struct
{
    uint32_t Identifier;  /*!< CAN frame identifier (11-bit standard or 29-bit extended) */
    uint32_t Control;     /*!< CAN frame control (IDE, RTR, DLC) */
    uint32_t Data[2];     /*!< CAN frame data (up to 8 bytes) */
} HAL_CAN_FrameTypeDef;

/********************* Bit definition for Frame Control register *********************/
#define CAN_FRAME_CONTROL_DLC_Pos               (0U)
#define CAN_FRAME_CONTROL_DLC_Msk               (0xFUL << CAN_FRAME_CONTROL_DLC_Pos)
#define CAN_FRAME_CONTROL_DLC                   CAN_FRAME_CONTROL_DLC_Msk
#define CAN_FRAME_CONTROL_RTR_Pos               (6U)
#define CAN_FRAME_CONTROL_RTR_Msk               (0x1UL << CAN_FRAME_CONTROL_RTR_Pos)
#define CAN_FRAME_CONTROL_RTR                   CAN_FRAME_CONTROL_RTR_Msk
#define CAN_FRAME_CONTROL_IDE_Pos               (7U)
#define CAN_FRAME_CONTROL_IDE_Msk               (0x1UL << CAN_FRAME_CONTROL_IDE_Pos)
#define CAN_FRAME_CONTROL_IDE                   CAN_FRAME_CONTROL_IDE_Msk


/* Exported types ------------------------------------------------------------*/
/** @defgroup CAN_Exported_Types CAN Exported Types
 * @{
 */
/**
 * @brief  HAL State structures definition
 */
typedef enum
{
    HAL_CAN_STATE_RESET             = 0x00U,  /*!< CAN not yet initialized or disabled */
    HAL_CAN_STATE_READY             = 0x01U,  /*!< CAN initialized and ready for use   */
    HAL_CAN_STATE_LISTENING         = 0x02U,  /*!< CAN receive process is ongoing      */
    HAL_CAN_STATE_SLEEP_PENDING     = 0x03U,  /*!< CAN sleep request is pending        */
    HAL_CAN_STATE_SLEEP_ACTIVE      = 0x04U,  /*!< CAN sleep mode is active            */
    HAL_CAN_STATE_ERROR             = 0x05U   /*!< CAN error state                     */

} HAL_CAN_StateTypeDef;


/**
 * @brief  CAN init structure definition
 */
typedef struct
{
    uint32_t Prescaler;         /*!< Specifies the prescaler value for time quantum clock */

    uint32_t Mode;              /*!< Specifies the CAN operating mode.
                                     This parameter can be a value of @ref CAN_operating_mode */

    uint32_t SyncJumpWidth;     /*!< Specifies the maximum number of time quanta the CAN hardware
                                     is allowed to lengthen or shorten a bit to perform resynchronization. */

    uint32_t TimeSeg1;          /*!< Specifies the number of time quanta in Phase Segment 1 (Propagation segment inclusive) */

    uint32_t TimeSeg2;          /*!< Specifies the number of time quanta in Phase Segment 2. */


    // uint32_t FilterConfig[16];  /*!< Filter configuration array */
    // uint32_t FilterMask[16];    /*!< Filter mask array */
    uint32_t FilterEnable;      /*!< Filter enable bit */

} CAN_InitTypeDef;

/**
 * @brief  CAN filter configuration structure definition
 */
typedef struct
{
    uint32_t FilterId;              /*!< Specify the filter identifier (29 bits) */
    uint32_t FilterMask;            /*!< Specify the filter mask (29 bits) */

    uint32_t FilterBank;            /*!< Specify the filter bank to be initialized (0-15) */

    uint32_t FilterActivation;      /*!< Enable or disable the filter */

    uint32_t IDECheckEnable;        /*!< IDE check enable (0=disable, 1=enable) */
    uint32_t IDEValue;              /*!< IDE value (0=standard frame only, 1=extended frame only) */

} CAN_FilterTypeDef;

/**
 * @brief  CAN Tx message header structure definition
 */
typedef struct
{
    uint32_t StdId;    /*!< Specifies the standard identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF. */

    uint32_t ExtId;    /*!< Specifies the extended identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF. */

    uint8_t IDE;      /*!< Specifies the type of identifier for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_identifier_type */

    uint8_t RTR;      /*!< Specifies the type of frame for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_remote_transmission_request */

    uint8_t DLC;      /*!< Specifies the length of the frame that will be transmitted.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 8. */

} CAN_TxHeaderTypeDef ;

/**
 * @brief  CAN Rx message header structure definition
 */
typedef struct
{
    uint32_t StdId;    /*!< Specifies the standard identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF. */

    uint32_t ExtId;    /*!< Specifies the extended identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF. */

    uint8_t IDE;      /*!< Specifies the type of identifier for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_identifier_type */

    uint8_t RTR;      /*!< Specifies the type of frame for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_remote_transmission_request */

    uint8_t DLC;      /*!< Specifies the length of the frame that will be transmitted.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 8. */
    uint8_t  Overflow;/*!< Overflow flag: 1=frame overwritten, 0=normal */

} CAN_RxHeaderTypeDef;

/**
 * @brief  CAN handle Structure definition
 */
typedef struct __CAN_HandleTypeDef
{
    CAN_TypeDef                 *Instance;                 /*!< Register base address */

    CAN_InitTypeDef             Init;                      /*!< CAN required parameters */

    __IO HAL_CAN_StateTypeDef   State;                     /*!< CAN communication state */

    __IO uint32_t               ErrorCode;                 /*!< CAN Error code.
                                                              This parameter can be a value of @ref CAN_Error_Code */
    uint8_t                     NumOfTxMsgs;               /*!< Number of messages pending transmission in secondary buffer */

} CAN_HandleTypeDef;

/**
 * @}
 */

/* Exported constants --------------------------------------------------------*/

/** @defgroup CAN_Exported_Constants CAN Exported Constants
 * @{
 */

/** @defgroup CAN_Error_Code CAN Error Code
 * @{
 */
#define HAL_CAN_ERROR_NONE            (0x00000000U)  /*!< No error                                             */
#define HAL_CAN_ERROR_EWG             (0x00000001U)  /*!< Protocol Error Warning                               */
#define HAL_CAN_ERROR_EPV             (0x00000002U)  /*!< Error Passive                                        */
#define HAL_CAN_ERROR_BOF             (0x00000004U)  /*!< Bus-off error                                        */
#define HAL_CAN_ERROR_STF             (0x00000008U)  /*!< Stuff error                                          */
#define HAL_CAN_ERROR_FOR             (0x00000010U)  /*!< Form error                                           */
#define HAL_CAN_ERROR_ACK             (0x00000020U)  /*!< Acknowledgment error                                 */
#define HAL_CAN_ERROR_BR              (0x00000040U)  /*!< Bit recessive error                                  */
#define HAL_CAN_ERROR_BD              (0x00000080U)  /*!< Bit dominant error                                   */
#define HAL_CAN_ERROR_CRC             (0x00000100U)  /*!< CRC error                                            */
#define HAL_CAN_ERROR_RX_FOV0         (0x00000200U)  /*!< Rx FIFO0 overrun error                               */
#define HAL_CAN_ERROR_RX_FOV1         (0x00000400U)  /*!< Rx FIFO1 overrun error                               */
#define HAL_CAN_ERROR_TX_ALST0        (0x00000800U)  /*!< TxMailbox 0 transmit failure due to arbitration lost */
#define HAL_CAN_ERROR_TX_TERR0        (0x00001000U)  /*!< TxMailbox 1 transmit failure due to tranmit error    */
#define HAL_CAN_ERROR_TX_ALST1        (0x00002000U)  /*!< TxMailbox 0 transmit failure due to arbitration lost */
#define HAL_CAN_ERROR_TX_TERR1        (0x00004000U)  /*!< TxMailbox 1 transmit failure due to tranmit error    */
#define HAL_CAN_ERROR_TX_ALST2        (0x00008000U)  /*!< TxMailbox 0 transmit failure due to arbitration lost */
#define HAL_CAN_ERROR_TX_TERR2        (0x00010000U)  /*!< TxMailbox 1 transmit failure due to tranmit error    */
#define HAL_CAN_ERROR_TIMEOUT         (0x00020000U)  /*!< Timeout error                                        */
#define HAL_CAN_ERROR_NOT_INITIALIZED (0x00040000U)  /*!< Peripheral not initialized                           */
#define HAL_CAN_ERROR_NOT_READY       (0x00080000U)  /*!< Peripheral not ready                                 */
#define HAL_CAN_ERROR_NOT_STARTED     (0x00100000U)  /*!< Peripheral not started                               */
#define HAL_CAN_ERROR_PARAM           (0x00200000U)  /*!< Parameter error                                      */

/**
 * @}
 */

/** @defgroup CAN_InitStatus CAN InitStatus
 * @{
 */
#define CAN_INITSTATUS_FAILED       (0x00000000U)  /*!< CAN initialization failed */
#define CAN_INITSTATUS_SUCCESS      (0x00000001U)  /*!< CAN initialization OK     */
/**
 * @}
 */

/** @defgroup CAN_operating_mode CAN Operating Mode
 * @{
 */
//TODO:
#define CAN_MODE_NORMAL             (0x00000000U)                   /*!< Normal mode   */
#define CAN_MODE_LOOPBACK           ((uint32_t)0)                   /*!< Loopback mode */
#define CAN_MODE_SILENT             ((uint32_t)CAN_CR_LOM)          /*!< Silent mode */
#define CAN_MODE_SILENT_LOOPBACK    ((uint32_t)0)                   /*!< Loopback combined with silent mode */
/**
 * @}
 */


/** @defgroup CAN_filter_mode CAN Filter Mode
 * @{
 */
#define CAN_FILTERMODE_IDMASK       (0x00000000U)  /*!< Identifier mask mode */
#define CAN_FILTERMODE_IDLIST       (0x00000001U)  /*!< Identifier list mode */

/**
 * @}
 */

/** @defgroup CAN_filter_scale CAN Filter Scale
 * @{
 */
#define CAN_FILTERSCALE_16BIT       (0x00000000U)  /*!< Two 16-bit filters */
#define CAN_FILTERSCALE_32BIT       (0x00000001U)  /*!< One 32-bit filter  */
/**
 * @}
 */

/** @defgroup CAN_filter_activation CAN Filter Activation
 * @{
 */
#define CAN_FILTER_DISABLE          (0x00000000U)  /*!< Disable filter */
#define CAN_FILTER_ENABLE           (0x00000001U)  /*!< Enable filter  */
/**
 * @}
 */

/** @defgroup CAN_filter_FIFO CAN Filter FIFO
 * @{
 */
#define CAN_FILTER_FIFO0            (0x00000000U)  /*!< Filter FIFO 0 assignment for filter x */
#define CAN_FILTER_FIFO1            (0x00000001U)  /*!< Filter FIFO 1 assignment for filter x */
/**
 * @}
 */

/** @defgroup CAN_identifier_type CAN Identifier Type
 * @{
 */
#define CAN_ID_STD                  0x00U                 /*!< Standard Id */
#define CAN_ID_EXT                  CAN_FRAME_CONTROL_IDE /*!< Extended Id */
/**
 * @}
 */

/** @defgroup CAN_remote_transmission_request CAN Remote Transmission Request
 * @{
 */
#define CAN_RTR_DATA                0x00U                  /*!< Data frame   */
#define CAN_RTR_REMOTE              CAN_FRAME_CONTROL_RTR  /*!< Remote frame */
/**
 * @}
 */


/** @defgroup CAN_Interrupts CAN Interrupts
 * @{
 */
/* Transmit Interrupt */
#define CAN_IT_TX_STB_EMPTY         (CAN_IR_TSIE)   /*!< STB transmit success interrupt         */
#define CAN_IT_TX_PTB_EMPTY         (CAN_IR_TPIE)   /*!< PTB transmit success interrupt         */

/* Receive Interrupts */
#define CAN_IT_RX_MSG_PENDING       (CAN_IR_RIE)    /*!< RX FIFO not empty interrupt            */
#define CAN_IT_RX_FIFO_FULL         (CAN_IR_RFIE)   /*!< RX FIFO full interrupt                 */
#define CAN_IT_RX_OVERRUN           (CAN_IR_ROIE)   /*!< RX FIFO overflow interrupt             */
#define CAN_IT_RX_ALMOST_FULL       (CAN_IR_RAFIE)  /*!< RX FIFO almost full interrupt          */

/* Error Interrupts */
#define CAN_IT_ERROR_WARNING        (0U)            /*!< Error warning interrupt                */
#define CAN_IT_ERROR_PASSIVE        (CAN_IR_EPIE)   /*!< Error passive interrupt                */
#define CAN_IT_BUSOFF               (CAN_IR_BEIE)   /*!< Error interrupt (bus-off included)     */
#define CAN_IT_LAST_ERROR_CODE      (0U)            /*!< Last error code (not supported)        */
#define CAN_IT_ERROR                (CAN_IR_EIE)    /*!< Error interrupt                        */
#define CAN_IT_ARBITRATION_LOST_ERROR    (CAN_IR_ALIE)    /*!< Error arbitration lost interrupt */

/**
 * @}
 */

/**
 * @}
 */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup CAN_Exported_Macros CAN Exported Macros
 * @{
 */

/** @brief  Reset CAN handle state
 * @param  __HANDLE__ CAN handle.
 * @retval None
 */
#define __HAL_CAN_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_CAN_STATE_RESET)

/**
 * @brief  Enable the specified CAN interrupts.
 * @param  __HANDLE__ CAN handle.
 * @param  __INTERRUPT__ CAN Interrupt sources to enable.
 *           This parameter can be any combination of @arg CAN_Interrupts
 * @retval None
 */
#define __HAL_CAN_ENABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->IR) |= (__INTERRUPT__))

/**
 * @brief  Disable the specified CAN interrupts.
 * @param  __HANDLE__ CAN handle.
 * @param  __INTERRUPT__ CAN Interrupt sources to disable.
 *           This parameter can be any combination of @arg CAN_Interrupts
 * @retval None
 */
#define __HAL_CAN_DISABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->IR) &= ~(__INTERRUPT__))

/** @brief  Check if the specified CAN interrupt source is enabled or disabled.
 * @param  __HANDLE__ specifies the CAN Handle.
 * @param  __INTERRUPT__ specifies the CAN interrupt source to check.
 *           This parameter can be a value of @arg CAN_Interrupts
 * @retval The state of __IT__ (TRUE or FALSE).
 */
#define __HAL_CAN_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->IR) & (__INTERRUPT__))

/**
 * @}
 */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup CAN_Exported_Functions CAN Exported Functions
 * @{
 */

/** @addtogroup CAN_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 * @{
 */

/* Initialization and de-initialization functions *****************************/
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_DeInit(CAN_HandleTypeDef *hcan);
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan);
void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan);

/**
 * @}
 */

/** @addtogroup CAN_Exported_Functions_Group2 Configuration functions
 *  @brief    Configuration functions
 * @{
 */

/* Configuration functions ****************************************************/
/**
 * @brief  Configures the CAN reception filter according to the specified
 *         parameters in the CAN_FilterInitStruct.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param  sFilterConfig pointer to a CAN_FilterTypeDef structure that
 *         contains the filter configuration information.
 * @retval None
 */
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig);

/**
 * @brief  Calculates the CAN timing parameters according to the specified bitrate.
 *
 * @param  hcan pointer to a CAN_HandleTypeDef structure,
 *              timing parameters in hcan->Init such as Prescaler and Seg1/Seg2 will be updated
 * @param  bitrate the desired bitrate for the CAN communication in bits/second
 *
 * @retval 0 or positive sample point error on success.
 * @retval -HAL_ERROR if the requested bitrate or sample point is out of range.
 */
int32_t HAL_CAN_CalcTiming(CAN_HandleTypeDef *hcan, uint32_t bitrate);

/**
 * @}
 */



/** @addtogroup CAN_Exported_Functions_Group3 Control functions
 *  @brief    Control functions
 * @{
 */

/* Control functions **********************************************************/
/**
 * @brief  Start the CAN module.
 * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan);

/**
 * @brief  Stop the CAN module and enable access to configuration registers.
 * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan);

/**
 * @brief  Add a message to the primary tx fifo and activate the
 *         transmission request.
 *
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param  pHeader pointer to a CAN_TxHeaderTypeDef structure.
 * @param  aData array containing the payload of the Tx frame.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_CAN_AddPrimaryTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[]);

/**
 * @brief  Add a message to the secondary tx fifo and activate
 *         the transmission request.
 *
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param  pHeader pointer to a CAN_TxHeaderTypeDef structure.
 * @param  aData array containing the payload of the Tx frame.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_CAN_AddSecondaryTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[]);

/**
 * @brief  Abort transmission requests
 *
 * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param  primary boolean indicating whether to abort primary tx fifo tranmission, true: primary, false: secondary.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan, bool primary);

/**
 * @brief  Return Tx FIFO free level: number of free Tx FIFO entries.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param  primary boolean indicating whether to check primary fifo, true: primary, false: secondary.
 * @retval Number of free Tx FIFO entries.
 */
uint32_t HAL_CAN_GetTxFifoFreeLevel(CAN_HandleTypeDef *hcan, bool primary);

/**
 * @brief  Check if a transmission request is pending
 *
 * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval Status
 *          - 0 : No pending transmission request on any selected Tx FIFOs.
 *          - 1 : Pending transmission request on primary or secondary Tx FIFO.
 */
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *hcan);

/**
 * @brief  Get an CAN frame from the Rx FIFO zone into the message RAM.
 * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param  RxFifo Fifo number of the received message to be read.
 *         This parameter can be a value of @arg CAN_receive_FIFO_number.
 * @param  pHeader pointer to a CAN_RxHeaderTypeDef structure where the header
 *         of the Rx frame will be stored.
 * @param  aData array where the payload of the Rx frame will be stored.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);

/**
 * @brief  Return Rx FIFO fill level.
 * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param  RxFifo Rx FIFO.
 *         This parameter can be a value of @arg CAN_receive_FIFO_number.
 * @return Rx FIFO fill level, value: 0 or 1
 * @retval 0: no messages available in Rx FIFO
 * @retval 1: message available in Rx FIFO
 */
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *hcan);

/**
 * @}
 */

/** @addtogroup CAN_Exported_Functions_Group4 Interrupts management
 *  @brief    Interrupts management
 * @{
 */
/* Interrupts management ******************************************************/
/**
 * @brief  Enable interrupts.
 * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param  ActiveITs indicates which interrupts will be enabled.
 *         This parameter can be any combination of CAN_IR_xxxE bits:
 *         CAN_IR_TSFF, CAN_IR_EIE, CAN_IR_TSIE, CAN_IR_TPIE,
 *         CAN_IR_RAFIE, CAN_IR_RFIE, CAN_IR_ROIE, CAN_IR_RIE.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs);

/**
 * @brief  Disable interrupts.
 * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param  InactiveITs indicates which interrupts will be disabled.
 *         This parameter can be any combination of CAN_IR_xxxE bits.
 * @retval HAL status
 */

HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *hcan, uint32_t InactiveITs);

/**
 * @brief  Handles CAN interrupt request.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan);

/**
 * @}
 */

/** @addtogroup CAN_Exported_Functions_Group5 Callback functions
 *  @brief    Callback functions
 * @{
 */
/* Callbacks functions ********************************************************/

/**
 * @brief  Primary TX FIFO transmission complete callback.
 *
 *  Weak function that can be overridden by the user to handle primary TX FIFO transmission complete events.
 *
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_TxPrimaryCompleteCallback(CAN_HandleTypeDef *hcan);

/**
 * @brief  Secondary TX FIFO transmission complete callback.
 *
 *  Weak function that can be overridden by the user to handle secondary TX FIFO transmission complete events.
 *
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_TxSecondaryCompleteCallback(CAN_HandleTypeDef *hcan);

/**
 * @brief  Rx msg pending callback.
 *
 * Weak function that can be overridden by the user to handle Rx message pending events.
 *
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxMsgPendingCallback(CAN_HandleTypeDef *hcan);

/**
 * @brief  Rx FIFO full callback.
 *
 * Weak function that can be overridden by the user to handle Rx FIFO full events.
 *
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxFifoFullCallback(CAN_HandleTypeDef *hcan);

/**
 * @brief  Rx almost full callback.
 *
 * Weak function that can be overridden by the user to handle Rx FIFO almost full events.
 *
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxFifoAlmostFullCallback(CAN_HandleTypeDef *hcan);

/**
 * @brief  Rx overflow callback.
 *
 * Weak function that can be overridden by the user to handle Rx FIFO overflow events.
 *
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxFifoOverflowCallback(CAN_HandleTypeDef *hcan);

/**
 * @brief  Error CAN callback.
 *
 * Weak function that can be overridden by the user to handle CAN error events.
 *
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);

/**
 * @}
 */

/**
 * @}
 */


/* Private Macros -----------------------------------------------------------*/
/** @defgroup CAN_Private_Macros CAN Private Macros
 * @{
 */

#define IS_CAN_PRESCALER(PRESCALER) (((PRESCALER) >= 1U) && ((PRESCALER) <= 1024U))
#define IS_CAN_FILTER_ID_HALFWORD(HALFWORD) ((HALFWORD) <= 0xFFFFU)
#if   defined(CAN2)
#define IS_CAN_FILTER_BANK_DUAL(BANK) ((BANK) <= 27U)
#endif
#define IS_CAN_FILTER_BANK_SINGLE(BANK) ((BANK) <= 13U)
#define IS_CAN_FILTER_MODE(MODE) (((MODE) == CAN_FILTERMODE_IDMASK) || \
                                  ((MODE) == CAN_FILTERMODE_IDLIST))
#define IS_CAN_FILTER_SCALE(SCALE) (((SCALE) == CAN_FILTERSCALE_16BIT) || \
                                    ((SCALE) == CAN_FILTERSCALE_32BIT))
#define IS_CAN_FILTER_ACTIVATION(ACTIVATION) (((ACTIVATION) == CAN_FILTER_DISABLE) || \
                                              ((ACTIVATION) == CAN_FILTER_ENABLE))
#define IS_CAN_STDID(STDID)   ((STDID) <= 0x7FFU)
#define IS_CAN_EXTID(EXTID)   ((EXTID) <= 0x1FFFFFFFU)
#define IS_CAN_DLC(DLC)       ((DLC) <= 8U)
#define IS_CAN_IDTYPE(IDTYPE)  (((IDTYPE) == CAN_ID_STD) || \
                                ((IDTYPE) == CAN_ID_EXT))
#define IS_CAN_RTR(RTR) (((RTR) == CAN_RTR_DATA) || ((RTR) == CAN_RTR_REMOTE))
#define IS_CAN_IT(IT) (0 == ((IT) & ~(CAN_IT_TX_STB_EMPTY     | CAN_IT_TX_PTB_EMPTY      | \
                                      CAN_IT_RX_MSG_PENDING   | CAN_IT_RX_FIFO_FULL      | \
                                      CAN_IT_RX_OVERRUN       | CAN_IT_RX_ALMOST_FULL    | \
                                      CAN_IT_ERROR_PASSIVE    | CAN_IT_BUSOFF            | \
                                      CAN_IT_ERROR            | CAN_IT_ARBITRATION_LOST_ERROR)))




/**
 * @}
 */
/* End of private macros -----------------------------------------------------*/

/**
 * @}
 */


/**
 * @}
 */

#ifdef __cplusplus
}
#endif
#endif /* STM32F1xx_HAL_CAN_H */


