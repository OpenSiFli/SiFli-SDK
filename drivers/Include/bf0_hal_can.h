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

// CAN Handle Structure
typedef struct
{
    // CAN initialization configuration parameters
    uint32_t Prescaler;         /*!< Baud rate prescaler */
    uint32_t SJW;               /*!< Synchronous jump width */
    uint32_t TimeSeg1;               /*!< Time Period 1 */
    uint32_t TimeSeg2;               /*!< Time Period 2 */
    uint32_t Mode;              /*!< CAN working mode */

    // Filter configuration
    uint32_t FilterConfig[16];  /*!< Filter configuration array */
    uint32_t FilterMask[16];    /*!< Filter mask array */
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

#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
    void (* TxPtbCompleteCallback)(struct __CAN_HandleTypeDef *hcan); /*!< CAN PTB complete callback         */
    void (* TxStbCompleteCallback)(struct __CAN_HandleTypeDef *hcan); /*!< CAN STB complete callback         */
    void (* TxMailbox2CompleteCallback)(struct __CAN_HandleTypeDef *hcan);/*!< CAN Tx Mailbox 2 complete callback    */
    void (* TxMailbox0AbortCallback)(struct __CAN_HandleTypeDef *hcan);   /*!< CAN Tx Mailbox 0 abort callback       */
    void (* TxMailbox1AbortCallback)(struct __CAN_HandleTypeDef *hcan);   /*!< CAN Tx Mailbox 1 abort callback       */
    void (* TxMailbox2AbortCallback)(struct __CAN_HandleTypeDef *hcan);   /*!< CAN Tx Mailbox 2 abort callback       */
    void (* RxMsgPendingCallback)(struct __CAN_HandleTypeDef *hcan);/*!< CAN Rx msg pending callback         */
    void (* RxFifoFullCallback)(struct __CAN_HandleTypeDef *hcan);  /*!< CAN Rx FIFO full callback           */
    void (* RxAlmostFullCallback)(struct __CAN_HandleTypeDef *hcan);/*!< CAN Rx almost full callback         */
    void (* RxOverflowCallback)(struct __CAN_HandleTypeDef *hcan);  /*!< CAN Rx overflow callback            */
    void (* SleepCallback)(struct __CAN_HandleTypeDef *hcan);             /*!< CAN Sleep callback                    */
    void (* WakeUpFromRxMsgCallback)(struct __CAN_HandleTypeDef *hcan);   /*!< CAN Wake Up from Rx msg callback      */
    void (* ErrorCallback)(struct __CAN_HandleTypeDef *hcan);             /*!< CAN Error callback                    */

    void (* MspInitCallback)(struct __CAN_HandleTypeDef *hcan);           /*!< CAN Msp Init callback                 */
    void (* MspDeInitCallback)(struct __CAN_HandleTypeDef *hcan);         /*!< CAN Msp DeInit callback               */

#endif /* (USE_HAL_CAN_REGISTER_CALLBACKS) */
} CAN_HandleTypeDef;

#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
/**
  * @brief  HAL CAN common Callback ID enumeration definition
  */
typedef enum
{
    HAL_CAN_TX_PTB_COMPLETE_CB_ID            = 0x00U,    /*!< CAN PTB complete callback ID                     */
    HAL_CAN_TX_STB_COMPLETE_CB_ID            = 0x01U,    /*!< CAN STB complete callback ID                     */
    HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID       = 0x02U,    /*!< CAN Tx Mailbox 2 complete callback ID         */
    HAL_CAN_TX_MAILBOX0_ABORT_CB_ID          = 0x03U,    /*!< CAN Tx Mailbox 0 abort callback ID            */
    HAL_CAN_TX_MAILBOX1_ABORT_CB_ID          = 0x04U,    /*!< CAN Tx Mailbox 1 abort callback ID            */
    HAL_CAN_TX_MAILBOX2_ABORT_CB_ID          = 0x05U,    /*!< CAN Tx Mailbox 2 abort callback ID            */
    HAL_CAN_RX_MSG_PENDING_CB_ID             = 0x06U,    /*!< CAN Rx msg pending callback ID                   */
    HAL_CAN_RX_FIFO_FULL_CB_ID               = 0x07U,    /*!< CAN Rx FIFO full callback ID                     */
    HAL_CAN_RX_ALMOST_FULL_CB_ID             = 0x08U,    /*!< CAN Rx almost full callback ID                   */
    HAL_CAN_RX_OVERFLOW_CB_ID                = 0x09U,    /*!< CAN Rx overflow callback ID                      */
    HAL_CAN_SLEEP_CB_ID                      = 0x0AU,    /*!< CAN Sleep callback ID                         */
    HAL_CAN_WAKEUP_FROM_RX_MSG_CB_ID         = 0x0BU,    /*!< CAN Wake Up fropm Rx msg callback ID          */
    HAL_CAN_ERROR_CB_ID                      = 0x0CU,    /*!< CAN Error callback ID                         */

    HAL_CAN_MSPINIT_CB_ID                    = 0x0DU,    /*!< CAN MspInit callback ID                       */
    HAL_CAN_MSPDEINIT_CB_ID                  = 0x0EU,    /*!< CAN MspDeInit callback ID                     */

} HAL_CAN_CallbackIDTypeDef;

/**
  * @brief  HAL CAN Callback pointer definition
  */
typedef  void (*pCAN_CallbackTypeDef)(CAN_HandleTypeDef *hcan); /*!< pointer to a CAN callback function   */

#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
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

#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
#define HAL_CAN_ERROR_INVALID_CALLBACK (0x00400000U) /*!< Invalid Callback error                               */
#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
#define HAL_CAN_ERROR_INTERNAL        (0x00800000U)  /*!< Internal error                                       */

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
#define CAN_MODE_NORMAL             (0x00000000U)                              /*!< Normal mode   */
#define CAN_MODE_LOOPBACK           ((uint32_t)CAN_BTR_LBKM)                   /*!< Loopback mode */
#define CAN_MODE_SILENT             ((uint32_t)CAN_BTR_SILM)                   /*!< Silent mode   */
#define CAN_MODE_SILENT_LOOPBACK    ((uint32_t)(CAN_BTR_LBKM | CAN_BTR_SILM))  /*!< Loopback combined with silent mode */
/**
  * @}
  */


/** @defgroup CAN_synchronisation_jump_width CAN Synchronization Jump Width
  * @{
  */
#define CAN_SJW_1TQ                 (0x00000000U)              /*!< 1 time quantum */
#define CAN_SJW_2TQ                 ((uint32_t)CAN_BTR_SJW_0)  /*!< 2 time quantum */
#define CAN_SJW_3TQ                 ((uint32_t)CAN_BTR_SJW_1)  /*!< 3 time quantum */
#define CAN_SJW_4TQ                 ((uint32_t)CAN_BTR_SJW)    /*!< 4 time quantum */
/**
  * @}
  */

/** @defgroup CAN_time_quantum_in_bit_segment_1 CAN Time Quantum in Bit Segment 1
  * @{
  */
#define CAN_BS1_1TQ                 (0x00000000U)                                                /*!< 1 time quantum  */
#define CAN_BS1_2TQ                 ((uint32_t)CAN_BTR_TS1_0)                                    /*!< 2 time quantum  */
#define CAN_BS1_3TQ                 ((uint32_t)CAN_BTR_TS1_1)                                    /*!< 3 time quantum  */
#define CAN_BS1_4TQ                 ((uint32_t)(CAN_BTR_TS1_1 | CAN_BTR_TS1_0))                  /*!< 4 time quantum  */
#define CAN_BS1_5TQ                 ((uint32_t)CAN_BTR_TS1_2)                                    /*!< 5 time quantum  */
#define CAN_BS1_6TQ                 ((uint32_t)(CAN_BTR_TS1_2 | CAN_BTR_TS1_0))                  /*!< 6 time quantum  */
#define CAN_BS1_7TQ                 ((uint32_t)(CAN_BTR_TS1_2 | CAN_BTR_TS1_1))                  /*!< 7 time quantum  */
#define CAN_BS1_8TQ                 ((uint32_t)(CAN_BTR_TS1_2 | CAN_BTR_TS1_1 | CAN_BTR_TS1_0))  /*!< 8 time quantum  */
#define CAN_BS1_9TQ                 ((uint32_t)CAN_BTR_TS1_3)                                    /*!< 9 time quantum  */
#define CAN_BS1_10TQ                ((uint32_t)(CAN_BTR_TS1_3 | CAN_BTR_TS1_0))                  /*!< 10 time quantum */
#define CAN_BS1_11TQ                ((uint32_t)(CAN_BTR_TS1_3 | CAN_BTR_TS1_1))                  /*!< 11 time quantum */
#define CAN_BS1_12TQ                ((uint32_t)(CAN_BTR_TS1_3 | CAN_BTR_TS1_1 | CAN_BTR_TS1_0))  /*!< 12 time quantum */
#define CAN_BS1_13TQ                ((uint32_t)(CAN_BTR_TS1_3 | CAN_BTR_TS1_2))                  /*!< 13 time quantum */
#define CAN_BS1_14TQ                ((uint32_t)(CAN_BTR_TS1_3 | CAN_BTR_TS1_2 | CAN_BTR_TS1_0))  /*!< 14 time quantum */
#define CAN_BS1_15TQ                ((uint32_t)(CAN_BTR_TS1_3 | CAN_BTR_TS1_2 | CAN_BTR_TS1_1))  /*!< 15 time quantum */
#define CAN_BS1_16TQ                ((uint32_t)CAN_BTR_TS1) /*!< 16 time quantum */
/**
  * @}
  */

/** @defgroup CAN_time_quantum_in_bit_segment_2 CAN Time Quantum in Bit Segment 2
  * @{
  */
#define CAN_BS2_1TQ                 (0x00000000U)                                /*!< 1 time quantum */
#define CAN_BS2_2TQ                 ((uint32_t)CAN_BTR_TS2_0)                    /*!< 2 time quantum */
#define CAN_BS2_3TQ                 ((uint32_t)CAN_BTR_TS2_1)                    /*!< 3 time quantum */
#define CAN_BS2_4TQ                 ((uint32_t)(CAN_BTR_TS2_1 | CAN_BTR_TS2_0))  /*!< 4 time quantum */
#define CAN_BS2_5TQ                 ((uint32_t)CAN_BTR_TS2_2)                    /*!< 5 time quantum */
#define CAN_BS2_6TQ                 ((uint32_t)(CAN_BTR_TS2_2 | CAN_BTR_TS2_0))  /*!< 6 time quantum */
#define CAN_BS2_7TQ                 ((uint32_t)(CAN_BTR_TS2_2 | CAN_BTR_TS2_1))  /*!< 7 time quantum */
#define CAN_BS2_8TQ                 ((uint32_t)CAN_BTR_TS2)                      /*!< 8 time quantum */
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

/** @defgroup CAN_receive_FIFO_number CAN Receive FIFO Number
  * @{
  */
#define CAN_RX_FIFO0                (0x00000000U)  /*!< CAN receive FIFO 0 */
#define CAN_RX_FIFO1                (0x00000001U)  /*!< CAN receive FIFO 1 */
/**
  * @}
  */

/** @defgroup CAN_Tx_Mailboxes CAN Tx Mailboxes
  * @{
  */
#define CAN_TX_MAILBOX0             (0x00000001U)  /*!< Tx Mailbox 0  */
#define CAN_TX_MAILBOX1             (0x00000002U)  /*!< Tx Mailbox 1  */
#define CAN_TX_MAILBOX2             (0x00000004U)  /*!< Tx Mailbox 2  */
/**
  * @}
  */

/** @defgroup CAN_flags CAN Flags
  * @{
  */
/* Transmit Flags */
#define CAN_FLAG_RQCP0              (0x00000500U)  /*!< Request complete MailBox 0 flag   */
#define CAN_FLAG_TXOK0              (0x00000501U)  /*!< Transmission OK MailBox 0 flag    */
#define CAN_FLAG_ALST0              (0x00000502U)  /*!< Arbitration Lost MailBox 0 flag   */
#define CAN_FLAG_TERR0              (0x00000503U)  /*!< Transmission error MailBox 0 flag */
#define CAN_FLAG_RQCP1              (0x00000508U)  /*!< Request complete MailBox1 flag    */
#define CAN_FLAG_TXOK1              (0x00000509U)  /*!< Transmission OK MailBox 1 flag    */
#define CAN_FLAG_ALST1              (0x0000050AU)  /*!< Arbitration Lost MailBox 1 flag   */
#define CAN_FLAG_TERR1              (0x0000050BU)  /*!< Transmission error MailBox 1 flag */
#define CAN_FLAG_RQCP2              (0x00000510U)  /*!< Request complete MailBox2 flag    */
#define CAN_FLAG_TXOK2              (0x00000511U)  /*!< Transmission OK MailBox 2 flag    */
#define CAN_FLAG_ALST2              (0x00000512U)  /*!< Arbitration Lost MailBox 2 flag   */
#define CAN_FLAG_TERR2              (0x00000513U)  /*!< Transmission error MailBox 2 flag */
#define CAN_FLAG_TME0               (0x0000051AU)  /*!< Transmit mailbox 0 empty flag     */
#define CAN_FLAG_TME1               (0x0000051BU)  /*!< Transmit mailbox 1 empty flag     */
#define CAN_FLAG_TME2               (0x0000051CU)  /*!< Transmit mailbox 2 empty flag     */
#define CAN_FLAG_LOW0               (0x0000051DU)  /*!< Lowest priority mailbox 0 flag    */
#define CAN_FLAG_LOW1               (0x0000051EU)  /*!< Lowest priority mailbox 1 flag    */
#define CAN_FLAG_LOW2               (0x0000051FU)  /*!< Lowest priority mailbox 2 flag    */

/* Receive Flags */
#define CAN_FLAG_FF0                (0x00000203U)  /*!< RX FIFO 0 Full flag               */
#define CAN_FLAG_FOV0               (0x00000204U)  /*!< RX FIFO 0 Overrun flag            */
#define CAN_FLAG_FF1                (0x00000403U)  /*!< RX FIFO 1 Full flag               */
#define CAN_FLAG_FOV1               (0x00000404U)  /*!< RX FIFO 1 Overrun flag            */

/* Operating Mode Flags */
#define CAN_FLAG_INAK               (0x00000100U)  /*!< Initialization acknowledge flag   */
#define CAN_FLAG_SLAK               (0x00000101U)  /*!< Sleep acknowledge flag            */
#define CAN_FLAG_ERRI               (0x00000102U)  /*!< Error flag                        */
#define CAN_FLAG_WKU                (0x00000103U)  /*!< Wake up interrupt flag            */
#define CAN_FLAG_SLAKI              (0x00000104U)  /*!< Sleep acknowledge interrupt flag  */

/* Error Flags */
#define CAN_FLAG_EWG                (0x00000300U)  /*!< Error warning flag                */
#define CAN_FLAG_EPV                (0x00000301U)  /*!< Error passive flag                */
#define CAN_FLAG_BOF                (0x00000302U)  /*!< Bus-Off flag                      */
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
#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
#define __HAL_CAN_RESET_HANDLE_STATE(__HANDLE__) do{                                              \
                                                     (__HANDLE__)->State = HAL_CAN_STATE_RESET;   \
                                                     (__HANDLE__)->MspInitCallback = NULL;        \
                                                     (__HANDLE__)->MspDeInitCallback = NULL;      \
                                                   } while(0)
#else
#define __HAL_CAN_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_CAN_STATE_RESET)
#endif /*USE_HAL_CAN_REGISTER_CALLBACKS */

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

/** @brief  Check whether the specified CAN flag is set or not.
  * @param  __HANDLE__ specifies the CAN Handle.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of @arg CAN_flags
  * @retval The state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_CAN_GET_FLAG(__HANDLE__, __FLAG__) \
  ((((__FLAG__) >> 8U) == 5U)? ((((__HANDLE__)->Instance->TSR) & (1U << ((__FLAG__) & CAN_FLAG_MASK))) == (1U << ((__FLAG__) & CAN_FLAG_MASK))): \
   (((__FLAG__) >> 8U) == 2U)? ((((__HANDLE__)->Instance->RF0R) & (1U << ((__FLAG__) & CAN_FLAG_MASK))) == (1U << ((__FLAG__) & CAN_FLAG_MASK))): \
   (((__FLAG__) >> 8U) == 4U)? ((((__HANDLE__)->Instance->RF1R) & (1U << ((__FLAG__) & CAN_FLAG_MASK))) == (1U << ((__FLAG__) & CAN_FLAG_MASK))): \
   (((__FLAG__) >> 8U) == 1U)? ((((__HANDLE__)->Instance->MSR) & (1U << ((__FLAG__) & CAN_FLAG_MASK))) == (1U << ((__FLAG__) & CAN_FLAG_MASK))): \
   (((__FLAG__) >> 8U) == 3U)? ((((__HANDLE__)->Instance->ESR) & (1U << ((__FLAG__) & CAN_FLAG_MASK))) == (1U << ((__FLAG__) & CAN_FLAG_MASK))): 0U)

/** @brief  Clear the specified CAN pending flag.
  * @param  __HANDLE__ specifies the CAN Handle.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg CAN_FLAG_RQCP0: Request complete MailBox 0 Flag
  *            @arg CAN_FLAG_TXOK0: Transmission OK MailBox 0 Flag
  *            @arg CAN_FLAG_ALST0: Arbitration Lost MailBox 0 Flag
  *            @arg CAN_FLAG_TERR0: Transmission error MailBox 0 Flag
  *            @arg CAN_FLAG_RQCP1: Request complete MailBox 1 Flag
  *            @arg CAN_FLAG_TXOK1: Transmission OK MailBox 1 Flag
  *            @arg CAN_FLAG_ALST1: Arbitration Lost MailBox 1 Flag
  *            @arg CAN_FLAG_TERR1: Transmission error MailBox 1 Flag
  *            @arg CAN_FLAG_RQCP2: Request complete MailBox 2 Flag
  *            @arg CAN_FLAG_TXOK2: Transmission OK MailBox 2 Flag
  *            @arg CAN_FLAG_ALST2: Arbitration Lost MailBox 2 Flag
  *            @arg CAN_FLAG_TERR2: Transmission error MailBox 2 Flag
  *            @arg CAN_FLAG_FF0:   RX FIFO 0 Full Flag
  *            @arg CAN_FLAG_FOV0:  RX FIFO 0 Overrun Flag
  *            @arg CAN_FLAG_FF1:   RX FIFO 1 Full Flag
  *            @arg CAN_FLAG_FOV1:  RX FIFO 1 Overrun Flag
  *            @arg CAN_FLAG_WKUI:  Wake up Interrupt Flag
  *            @arg CAN_FLAG_SLAKI: Sleep acknowledge Interrupt Flag
  * @retval None
  */
#define __HAL_CAN_CLEAR_FLAG(__HANDLE__, __FLAG__) \
  ((((__FLAG__) >> 8U) == 5U)? (((__HANDLE__)->Instance->TSR) = (1U << ((__FLAG__) & CAN_FLAG_MASK))): \
   (((__FLAG__) >> 8U) == 2U)? (((__HANDLE__)->Instance->RF0R) = (1U << ((__FLAG__) & CAN_FLAG_MASK))): \
   (((__FLAG__) >> 8U) == 4U)? (((__HANDLE__)->Instance->RF1R) = (1U << ((__FLAG__) & CAN_FLAG_MASK))): \
   (((__FLAG__) >> 8U) == 1U)? (((__HANDLE__)->Instance->MSR) = (1U << ((__FLAG__) & CAN_FLAG_MASK))): 0U)

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

#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
/* Callbacks Register/UnRegister functions  ***********************************/
HAL_StatusTypeDef HAL_CAN_RegisterCallback(CAN_HandleTypeDef *hcan, HAL_CAN_CallbackIDTypeDef CallbackID, void (* pCallback)(CAN_HandleTypeDef *_hcan));
HAL_StatusTypeDef HAL_CAN_UnRegisterCallback(CAN_HandleTypeDef *hcan, HAL_CAN_CallbackIDTypeDef CallbackID);

#endif /* (USE_HAL_CAN_REGISTER_CALLBACKS) */
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
 * @brief  Add a message to the first free Tx mailbox and activate the
 *         corresponding transmission request.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param  pHeader pointer to a CAN_TxHeaderTypeDef structure.
 * @param  aData array containing the payload of the Tx frame.
 * @param  pTxMailbox pointer to a variable where the function will return
 *         the TxMailbox used to store the Tx message.
 *         This parameter can be a value of @arg CAN_Tx_Mailboxes.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[]);

/**
 * @brief  Add a message to the STB (Secondary Transmit Buffer) queue.
 *         Call this function multiple times to queue frames, then call
 *         HAL_CAN_STB_Transmit to send all queued frames at once.
 *
 *         Note: STB frames have lower priority than PTB frames. If PTB is
 *         triggered while STB frames are being sent, the hardware will
 *         automatically send the PTB frame first.
 *
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param  pHeader pointer to a CAN_TxHeaderTypeDef structure.
 * @param  aData array containing the payload of the Tx frame.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_CAN_STB_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[]);

/**
 * @brief  Abort transmission requests
 * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param  ptb boolean indicating whether to abort PTB requests.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan, bool primary);

/**
 * @brief  Return Tx Mailboxes free level: number of free Tx Mailboxes.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param  primary boolean indicating whether to check primary mailboxes.
 * @retval Number of free Tx Mailboxes.
 */
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *hcan, bool primary);

/**
 * @brief  Check if a transmission request is pending on the selected Tx
 *         Mailboxes.
 * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval Status
 *          - 0 : No pending transmission request on any selected Tx Mailboxes.
 *          - 1 : Pending transmission request on primary or secondary Tx Mailbox.
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
 * @brief  PTB transmission complete callback.
 *
 *  Weak function that can be overridden by the user to handle PTB transmission complete events.
 *
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_TxPtbCompleteCallback(CAN_HandleTypeDef *hcan);

/**
 * @brief  STB transmission complete callback.
 *
 *  Weak function that can be overridden by the user to handle STB transmission complete events.
 *
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_TxStbCompleteCallback(CAN_HandleTypeDef *hcan);

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
void HAL_CAN_RxAlmostFullCallback(CAN_HandleTypeDef *hcan);

/**
 * @brief  Rx overflow callback.
 *
 * Weak function that can be overridden by the user to handle Rx FIFO overflow events.
 *
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxOverflowCallback(CAN_HandleTypeDef *hcan);

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

/* Private types -------------------------------------------------------------*/
/** @defgroup CAN_Private_Types CAN Private Types
  * @{
  */

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup CAN_Private_Variables CAN Private Variables
  * @{
  */

/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup CAN_Private_Constants CAN Private Constants
  * @{
  */
#define CAN_FLAG_MASK  (0x000000FFU)
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


