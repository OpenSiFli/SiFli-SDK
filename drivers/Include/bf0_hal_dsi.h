/**
  ******************************************************************************
  * @file   bf0_hal_dsi.h
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
#ifndef BF0_HAL_DSI_H
#define BF0_HAL_DSI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup DSI DSI
  * @brief DSI HAL module driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup DSI_Exported_Types DSI Exported Types
  * @brief
  * @{
  */
/**
  * @brief  DSI Init Structure definition
  */
typedef struct
{
    uint32_t AutomaticClockLaneControl;    /*!< Automatic clock lane control
                                              This parameter can be any value of @ref DSI_Automatic_Clk_Lane_Control */

    uint32_t TXEscapeCkdiv;                /*!< TX Escape clock division
                                              The values 0 and 1 stop the TX_ESC clock generation                    */

    uint32_t NumberOfLanes;                /*!< Number of lanes
                                              This parameter can be any value of @ref DSI_Number_Of_Lanes            */

} DSI_InitTypeDef;

/**
  * @brief  DSI DLL Clock structure definition
  */
typedef struct
{

    uint32_t STG;

    uint32_t output_div;
} DSI_DLLInitTypeDef;

/**
  * @brief  DSI Video mode configuration
  */
typedef struct
{
    uint32_t VirtualChannelID;             /*!< Virtual channel ID                                                 */

    uint32_t ColorCoding;                  /*!< Color coding for LTDC interface
                                              This parameter can be any value of @ref DSI_Color_Coding           */

    uint32_t LooselyPacked;                /*!< Enable or disable loosely packed stream (needed only when using
                                              18-bit configuration).
                                              This parameter can be any value of @ref DSI_LooselyPacked          */

    uint32_t Mode;                         /*!< Video mode type
                                              This parameter can be any value of @ref DSI_Video_Mode_Type        */

    uint32_t PacketSize;                   /*!< Video packet size                                                  */

    uint32_t NumberOfChunks;               /*!< Number of chunks                                                   */

    uint32_t NullPacketSize;               /*!< Null packet size                                                   */

    uint32_t HSPolarity;                   /*!< HSYNC pin polarity
                                              This parameter can be any value of @ref DSI_HSYNC_Polarity         */

    uint32_t VSPolarity;                   /*!< VSYNC pin polarity
                                              This parameter can be any value of @ref DSI_VSYNC_Active_Polarity  */

    uint32_t DEPolarity;                   /*!< Data Enable pin polarity
                                              This parameter can be any value of @ref DSI_DATA_ENABLE_Polarity   */

    uint32_t HorizontalSyncActive;         /*!< Horizontal synchronism active duration (in lane byte clock cycles) */

    uint32_t HorizontalBackPorch;          /*!< Horizontal back-porch duration (in lane byte clock cycles)         */

    uint32_t HorizontalLine;               /*!< Horizontal line duration (in lane byte clock cycles)               */

    uint32_t VerticalSyncActive;           /*!< Vertical synchronism active duration                               */

    uint32_t VerticalBackPorch;            /*!< Vertical back-porch duration                                       */

    uint32_t VerticalFrontPorch;           /*!< Vertical front-porch duration                                      */

    uint32_t VerticalActive;               /*!< Vertical active duration                                           */

    uint32_t LPCommandEnable;              /*!< Low-power command enable
                                              This parameter can be any value of @ref DSI_LP_Command             */

    uint32_t LPLargestPacketSize;          /*!< The size, in bytes, of the low power largest packet that
                                              can fit in a line during VSA, VBP and VFP regions                  */

    uint32_t LPVACTLargestPacketSize;      /*!< The size, in bytes, of the low power largest packet that
                                              can fit in a line during VACT region                               */

    uint32_t LPHorizontalFrontPorchEnable; /*!< Low-power horizontal front-porch enable
                                              This parameter can be any value of @ref DSI_LP_HFP                 */

    uint32_t LPHorizontalBackPorchEnable;  /*!< Low-power horizontal back-porch enable
                                              This parameter can be any value of @ref DSI_LP_HBP                 */

    uint32_t LPVerticalActiveEnable;       /*!< Low-power vertical active enable
                                              This parameter can be any value of @ref DSI_LP_VACT                */

    uint32_t LPVerticalFrontPorchEnable;   /*!< Low-power vertical front-porch enable
                                              This parameter can be any value of @ref DSI_LP_VFP                 */

    uint32_t LPVerticalBackPorchEnable;    /*!< Low-power vertical back-porch enable
                                              This parameter can be any value of @ref DSI_LP_VBP                 */

    uint32_t LPVerticalSyncActiveEnable;   /*!< Low-power vertical sync active enable
                                              This parameter can be any value of @ref DSI_LP_VSYNC               */

    uint32_t FrameBTAAcknowledgeEnable;    /*!< Frame bus-turn-around acknowledge enable
                                              This parameter can be any value of @ref DSI_FBTA_acknowledge       */

} DSI_VidCfgTypeDef;

/**
  * @brief  DSI Adapted command mode configuration
  */
typedef struct
{
    uint32_t VirtualChannelID;             /*!< Virtual channel ID                                                */

    uint32_t ColorCoding;                  /*!< Color coding for LTDC interface
                                              This parameter can be any value of @ref DSI_Color_Coding          */

    uint32_t CommandSize;                  /*!< Maximum allowed size for an LTDC write memory command, measured in
                                              pixels. This parameter can be any value between 0x00 and 0xFFFFU   */

    uint32_t TearingEffectSource;          /*!< Tearing effect source
                                              This parameter can be any value of @ref DSI_TearingEffectSource   */

    uint32_t TearingEffectPolarity;        /*!< Tearing effect pin polarity
                                              This parameter can be any value of @ref DSI_TearingEffectPolarity */

    uint32_t HSPolarity;                   /*!< HSYNC pin polarity
                                              This parameter can be any value of @ref DSI_HSYNC_Polarity        */

    uint32_t VSPolarity;                   /*!< VSYNC pin polarity
                                              This parameter can be any value of @ref DSI_VSYNC_Active_Polarity */

    uint32_t DEPolarity;                   /*!< Data Enable pin polarity
                                              This parameter can be any value of @ref DSI_DATA_ENABLE_Polarity  */

    uint32_t VSyncPol;                     /*!< VSync edge on which the LTDC is halted
                                              This parameter can be any value of @ref DSI_Vsync_Polarity        */

    uint32_t AutomaticRefresh;             /*!< Automatic refresh mode
                                              This parameter can be any value of @ref DSI_AutomaticRefresh      */

    uint32_t TEAcknowledgeRequest;         /*!< Tearing Effect Acknowledge Request Enable
                                              This parameter can be any value of @ref DSI_TE_AcknowledgeRequest */

} DSI_CmdCfgTypeDef;

/**
  * @brief  DSI command transmission mode configuration
  */
typedef struct
{
    uint32_t LPGenShortWriteNoP;           /*!< Generic Short Write Zero parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortWriteNoP  */

    uint32_t LPGenShortWriteOneP;          /*!< Generic Short Write One parameter Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortWriteOneP */

    uint32_t LPGenShortWriteTwoP;          /*!< Generic Short Write Two parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortWriteTwoP */

    uint32_t LPGenShortReadNoP;            /*!< Generic Short Read Zero parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortReadNoP   */

    uint32_t LPGenShortReadOneP;           /*!< Generic Short Read One parameter Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortReadOneP  */

    uint32_t LPGenShortReadTwoP;           /*!< Generic Short Read Two parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortReadTwoP  */

    uint32_t LPGenLongWrite;               /*!< Generic Long Write Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenLongWrite      */

    uint32_t LPDcsShortWriteNoP;           /*!< DCS Short Write Zero parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPDcsShortWriteNoP  */

    uint32_t LPDcsShortWriteOneP;          /*!< DCS Short Write One parameter Transmission
                                              This parameter can be any value of @ref DSI_LP_LPDcsShortWriteOneP */

    uint32_t LPDcsShortReadNoP;            /*!< DCS Short Read Zero parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPDcsShortReadNoP   */

    uint32_t LPDcsLongWrite;               /*!< DCS Long Write Transmission
                                              This parameter can be any value of @ref DSI_LP_LPDcsLongWrite      */

    uint32_t LPMaxReadPacket;              /*!< Maximum Read Packet Size Transmission
                                              This parameter can be any value of @ref DSI_LP_LPMaxReadPacket     */

    uint32_t AcknowledgeRequest;           /*!< Acknowledge Request Enable
                                              This parameter can be any value of @ref DSI_AcknowledgeRequest     */

} DSI_LPCmdTypeDef;

/**
  * @brief  DSI PHY Timings definition
  */
typedef struct
{
    uint32_t ClockLaneHS2LPTime;           /*!< The maximum time that the D-PHY clock lane takes to go from high-speed
                                              to low-power transmission                                              */

    uint32_t ClockLaneLP2HSTime;           /*!< The maximum time that the D-PHY clock lane takes to go from low-power
                                              to high-speed transmission                                             */

    uint32_t DataLaneHS2LPTime;            /*!< The maximum time that the D-PHY data lanes takes to go from high-speed
                                              to low-power transmission                                              */

    uint32_t DataLaneLP2HSTime;            /*!< The maximum time that the D-PHY data lanes takes to go from low-power
                                              to high-speed transmission                                             */

    uint32_t DataLaneMaxReadTime;          /*!< The maximum time required to perform a read command */

    uint32_t StopWaitTime;                 /*!< The minimum wait period to request a High-Speed transmission after the
                                              Stop state                                                             */

} DSI_PHY_TimerTypeDef;

/**
  * @brief  DSI HOST Timeouts definition
  */
typedef struct
{
    uint32_t TimeoutCkdiv;                 /*!< Time-out clock division                                  */

    uint32_t HighSpeedTransmissionTimeout; /*!< High-speed transmission time-out                         */

    uint32_t LowPowerReceptionTimeout;     /*!< Low-power reception time-out                             */

    uint32_t HighSpeedReadTimeout;         /*!< High-speed read time-out                                 */

    uint32_t LowPowerReadTimeout;          /*!< Low-power read time-out                                  */

    uint32_t HighSpeedWriteTimeout;        /*!< High-speed write time-out                                */

    //uint32_t HighSpeedWritePrespMode;      /*!< High-speed write presp mode
    // NOT USE                                            This parameter can be any value of @ref DSI_HS_PrespMode */

    uint32_t LowPowerWriteTimeout;         /*!< Low-speed write time-out                                 */

    uint32_t BTATimeout;                   /*!< BTA time-out                                             */

} DSI_HOST_TimeoutTypeDef;

/**
  * @brief  DSI States Structure definition
  */
typedef enum
{
    HAL_DSI_STATE_RESET   = 0x00U,
    HAL_DSI_STATE_READY   = 0x01U,
    HAL_DSI_STATE_ERROR   = 0x02U,
    HAL_DSI_STATE_BUSY    = 0x03U,
    HAL_DSI_STATE_TIMEOUT = 0x04U
} HAL_DSI_StateTypeDef;

/**
  * @brief  DSI Handle Structure definition
  */
typedef struct
{
    DSI_HOST_TypeDef          *Instance;    /*!< Host register base address      */
    DSI_PHY_TypeDef           *PHY_Instance;     /*!< PHY register base address      */
    DSI_InitTypeDef           Init;         /*!< DSI required parameters    */
    HAL_LockTypeDef           Lock;         /*!< DSI peripheral status      */
    __IO HAL_DSI_StateTypeDef State;        /*!< DSI communication state    */
    __IO uint32_t             ErrorCode;    /*!< DSI Error code             */
    uint32_t                  ErrorMsk;     /*!< DSI Error monitoring mask  */
} DSI_HandleTypeDef;
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup DSI_Exported_Constants DSI Exported Constants
  * @brief
  * @{
  */

/** @defgroup DSI_DCS_Command DSI DCS Command
  * @{
  */
#define DSI_ENTER_IDLE_MODE       0x39U
#define DSI_ENTER_INVERT_MODE     0x21U
#define DSI_ENTER_NORMAL_MODE     0x13U
#define DSI_ENTER_PARTIAL_MODE    0x12U
#define DSI_ENTER_SLEEP_MODE      0x10U
#define DSI_EXIT_IDLE_MODE        0x38U
#define DSI_EXIT_INVERT_MODE      0x20U
#define DSI_EXIT_SLEEP_MODE       0x11U
#define DSI_GET_3D_CONTROL        0x3FU
#define DSI_GET_ADDRESS_MODE      0x0BU
#define DSI_GET_BLUE_CHANNEL      0x08U
#define DSI_GET_DIAGNOSTIC_RESULT 0x0FU
#define DSI_GET_DISPLAY_MODE      0x0DU
#define DSI_GET_GREEN_CHANNEL     0x07U
#define DSI_GET_PIXEL_FORMAT      0x0CU
#define DSI_GET_POWER_MODE        0x0AU
#define DSI_GET_RED_CHANNEL       0x06U
#define DSI_GET_SCANLINE          0x45U
#define DSI_GET_SIGNAL_MODE       0x0EU
#define DSI_NOP                   0x00U
#define DSI_READ_DDB_CONTINUE     0xA8U
#define DSI_READ_DDB_START        0xA1U
#define DSI_READ_MEMORY_CONTINUE  0x3EU
#define DSI_READ_MEMORY_START     0x2EU
#define DSI_SET_3D_CONTROL        0x3DU
#define DSI_SET_ADDRESS_MODE      0x36U
#define DSI_SET_COLUMN_ADDRESS    0x2AU
#define DSI_SET_DISPLAY_OFF       0x28U
#define DSI_SET_DISPLAY_ON        0x29U
#define DSI_SET_GAMMA_CURVE       0x26U
#define DSI_SET_PAGE_ADDRESS      0x2BU
#define DSI_SET_PARTIAL_COLUMNS   0x31U
#define DSI_SET_PARTIAL_ROWS      0x30U
#define DSI_SET_PIXEL_FORMAT      0x3AU
#define DSI_SET_SCROLL_AREA       0x33U
#define DSI_SET_SCROLL_START      0x37U
#define DSI_SET_TEAR_OFF          0x34U
#define DSI_SET_TEAR_ON           0x35U
#define DSI_SET_TEAR_SCANLINE     0x44U
#define DSI_SET_VSYNC_TIMING      0x40U
#define DSI_SOFT_RESET            0x01U
#define DSI_WRITE_LUT             0x2DU
#define DSI_WRITE_MEMORY_CONTINUE 0x3CU
#define DSI_WRITE_MEMORY_START    0x2CU
/**
  * @}
  */

/** @defgroup DSI_Video_Mode_Type DSI Video Mode Type
  * @{
  */
#define DSI_VID_MODE_NB_PULSES    0U
#define DSI_VID_MODE_NB_EVENTS    1U
#define DSI_VID_MODE_BURST        2U
/**
  * @}
  */

/** @defgroup DSI_Color_Mode DSI Color Mode
  * @{
  */
#define DSI_COLOR_MODE_FULL       0x00000000U
#define DSI_COLOR_MODE_EIGHT      DSI_WCR_COLM
/**
  * @}
  */

/** @defgroup DSI_ShutDown DSI ShutDown
  * @{
  */
#define DSI_DISPLAY_ON            0x00000000U
#define DSI_DISPLAY_OFF           DSI_WCR_SHTDN
/**
  * @}
  */

/** @defgroup DSI_LP_Command DSI LP Command
  * @{
  */
#define DSI_LP_COMMAND_DISABLE    0x00000000U
#define DSI_LP_COMMAND_ENABLE     DSI_HOST_VMC_LCMDE
/**
  * @}
  */

/** @defgroup DSI_LP_HFP DSI LP HFP
  * @{
  */
#define DSI_LP_HFP_DISABLE        0x00000000U
#define DSI_LP_HFP_ENABLE         DSI_HOST_VMC_LHFPE
/**
  * @}
  */

/** @defgroup DSI_LP_HBP DSI LP HBP
  * @{
  */
#define DSI_LP_HBP_DISABLE        0x00000000U
#define DSI_LP_HBP_ENABLE         DSI_HOST_VMC_LHBPE
/**
  * @}
  */

/** @defgroup DSI_LP_VACT DSI LP VACT
  * @{
  */
#define DSI_LP_VACT_DISABLE       0x00000000U
#define DSI_LP_VACT_ENABLE        DSI_HOST_VMC_LVACTE
/**
  * @}
  */

/** @defgroup DSI_LP_VFP DSI LP VFP
  * @{
  */
#define DSI_LP_VFP_DISABLE       0x00000000U
#define DSI_LP_VFP_ENABLE        DSI_HOST_VMC_LVFPE
/**
  * @}
  */

/** @defgroup DSI_LP_VBP DSI LP VBP
  * @{
  */
#define DSI_LP_VBP_DISABLE       0x00000000U
#define DSI_LP_VBP_ENABLE        DSI_HOST_VMC_LVBPE
/**
  * @}
  */

/** @defgroup DSI_LP_VSYNC DSI LP VSYNC
  * @{
  */
#define DSI_LP_VSYNC_DISABLE     0x00000000U
#define DSI_LP_VSYNC_ENABLE      DSI_HOST_VMC_LVSAE
/**
  * @}
  */

/** @defgroup DSI_FBTA_acknowledge DSI FBTA Acknowledge
  * @{
  */
#define DSI_FBTAA_DISABLE        0x00000000U
#define DSI_FBTAA_ENABLE         DSI_HOST_VMC_FBAE
/**
  * @}
  */

/** @defgroup DSI_TearingEffectSource DSI Tearing Effect Source
  * @{
  */
#define DSI_TE_DSILINK           0x00000000U
#define DSI_TE_EXTERNAL          0x00000001U
/**
  * @}
  */

/** @defgroup DSI_TearingEffectPolarity DSI Tearing Effect Polarity
  * @{
  */
#define DSI_TE_RISING_EDGE       0x00000000U
#define DSI_TE_FALLING_EDGE      DSI_WCFGR_TEPOL
/**
  * @}
  */

/** @defgroup DSI_Vsync_Polarity DSI Vsync Polarity
  * @{
  */
#define DSI_VSYNC_FALLING        0x00000000U
#define DSI_VSYNC_RISING         DSI_WCFGR_VSPOL
/**
  * @}
  */

/** @defgroup DSI_AutomaticRefresh DSI Automatic Refresh
  * @{
  */
#define DSI_AR_DISABLE           0x00000000U
#define DSI_AR_ENABLE            DSI_WCFGR_AR
/**
  * @}
  */


/** @defgroup DSI_TE_AcknowledgeRequest DSI TE Acknowledge Request
  * @{
  */
#define DSI_TE_ACKNOWLEDGE_DISABLE 0x00000000U
#define DSI_TE_ACKNOWLEDGE_ENABLE  DSI_HOST_CMODE_CFG_FMARK_EN
/**
  * @}
  */

/** @defgroup DSI_AcknowledgeRequest DSI Acknowledge Request
  * @{
  */
#define DSI_ACKNOWLEDGE_DISABLE   0x00000000U
#define DSI_ACKNOWLEDGE_ENABLE    DSI_HOST_CMODE_CFG_ACK_REQ_EN
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortWriteNoP DSI LP LPGen Short Write NoP
  * @{
  */
#define DSI_LP_GSW0P_DISABLE     0x00000000U
#define DSI_LP_GSW0P_ENABLE      DSI_HOST_CMODE_CFG_COM_SHORT_WR0
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortWriteOneP DSI LP LPGen Short Write OneP
  * @{
  */
#define DSI_LP_GSW1P_DISABLE     0x00000000U
#define DSI_LP_GSW1P_ENABLE      DSI_HOST_CMODE_CFG_COM_SHORT_WR1
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortWriteTwoP DSI LP LPGen Short Write TwoP
  * @{
  */
#define DSI_LP_GSW2P_DISABLE     0x00000000U
#define DSI_LP_GSW2P_ENABLE      DSI_HOST_CMODE_CFG_COM_SHORT_WR2
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortReadNoP DSI LP LPGen Short Read NoP
  * @{
  */
#define DSI_LP_GSR0P_DISABLE     0x00000000U
#define DSI_LP_GSR0P_ENABLE      DSI_HOST_CMODE_CFG_COM_SHORT_RD0
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortReadOneP DSI LP LPGen Short Read OneP
  * @{
  */
#define DSI_LP_GSR1P_DISABLE     0x00000000U
#define DSI_LP_GSR1P_ENABLE      DSI_HOST_CMODE_CFG_COM_SHORT_RD1
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortReadTwoP DSI LP LPGen Short Read TwoP
  * @{
  */
#define DSI_LP_GSR2P_DISABLE     0x00000000U
#define DSI_LP_GSR2P_ENABLE      DSI_HOST_CMODE_CFG_COM_SHORT_RD2
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenLongWrite DSI LP LPGen LongWrite
  * @{
  */
#define DSI_LP_GLW_DISABLE       0x00000000U
#define DSI_LP_GLW_ENABLE        DSI_HOST_CMODE_CFG_COM_LONG_WR
/**
  * @}
  */

/** @defgroup DSI_LP_LPDcsShortWriteNoP DSI LP LPDcs Short Write NoP
  * @{
  */
#define DSI_LP_DSW0P_DISABLE     0x00000000U
#define DSI_LP_DSW0P_ENABLE      DSI_HOST_CMODE_CFG_SHORT_WR0
/**
  * @}
  */

/** @defgroup DSI_LP_LPDcsShortWriteOneP DSI LP LPDcs Short Write OneP
  * @{
  */
#define DSI_LP_DSW1P_DISABLE     0x00000000U
#define DSI_LP_DSW1P_ENABLE      DSI_HOST_CMODE_CFG_SHORT_WR1
/**
  * @}
  */

/** @defgroup DSI_LP_LPDcsShortReadNoP DSI LP LPDcs Short Read NoP
  * @{
  */
#define DSI_LP_DSR0P_DISABLE     0x00000000U
#define DSI_LP_DSR0P_ENABLE      DSI_HOST_CMODE_CFG_SHORT_RD0
/**
  * @}
  */

/** @defgroup DSI_LP_LPDcsLongWrite DSI LP LPDcs Long Write
  * @{
  */
#define DSI_LP_DLW_DISABLE       0x00000000U
#define DSI_LP_DLW_ENABLE        DSI_HOST_CMODE_CFG_LONG_WR
/**
  * @}
  */

/** @defgroup DSI_LP_LPMaxReadPacket DSI LP LPMax Read Packet
  * @{
  */
#define DSI_LP_MRDP_DISABLE      0x00000000U
#define DSI_LP_MRDP_ENABLE       DSI_HOST_CMODE_CFG_MAX_RD_SIZE
/**
  * @}
  */

/** @defgroup DSI_HS_PrespMode DSI HS Presp Mode
  * @{
  */
#define DSI_HS_PM_DISABLE        0x00000000U
#define DSI_HS_PM_ENABLE         DSI_TCCR3_PM
/**
  * @}
  */


/** @defgroup DSI_Automatic_Clk_Lane_Control DSI Automatic Clk Lane Control
  * @{
  */
#define DSI_AUTO_CLK_LANE_CTRL_DISABLE 0x00000000U
#define DSI_AUTO_CLK_LANE_CTRL_ENABLE  DSI_HOST_PHY_CTRL_CKLANE_AUTO_CTRL
/**
  * @}
  */

/** @defgroup DSI_Number_Of_Lanes DSI Number Of Lanes
  * @{
  */
#define DSI_ONE_DATA_LANE          0U
#define DSI_TWO_DATA_LANES         1U
/**
  * @}
  */

/** @defgroup DSI_FlowControl DSI Flow Control
  * @{
  */
#define DSI_FLOW_CONTROL_CRC_RX    DSI_HOST_PKT_CFG_CRC_RX_EN
#define DSI_FLOW_CONTROL_ECC_RX    DSI_HOST_PKT_CFG_ECC_RX_EN
#define DSI_FLOW_CONTROL_BTA       DSI_HOST_PKT_CFG_BTA_EN
#define DSI_FLOW_CONTROL_EOTP_RX   DSI_HOST_PKT_CFG_EOTP_RX_EN
#define DSI_FLOW_CONTROL_EOTP_TX   DSI_HOST_PKT_CFG_EOTP_TX_EN
#define DSI_FLOW_CONTROL_ALL       (DSI_FLOW_CONTROL_CRC_RX | DSI_FLOW_CONTROL_ECC_RX | \
                                    DSI_FLOW_CONTROL_BTA | DSI_FLOW_CONTROL_EOTP_RX | \
                                    DSI_FLOW_CONTROL_EOTP_TX)
/**
  * @}
  */




/** @defgroup DSI_Color_Coding DSI Color Coding
  * @{
  */
#define DSI_RGB565                 0x00000000U /*!< The values 0x00000001 and 0x00000002 can also be used for the RGB565 color mode configuration */
#define DSI_RGB666                 0x00000003U /*!< The value 0x00000004 can also be used for the RGB666 color mode configuration                 */
#define DSI_RGB888                 0x00000005U
/**
  * @}
  */

/** @defgroup DSI_LooselyPacked DSI Loosely Packed
  * @{
  */
#define DSI_LOOSELY_PACKED_ENABLE  DSI_LCOLCR_LPE
#define DSI_LOOSELY_PACKED_DISABLE 0x00000000U
/**
  * @}
  */

/** @defgroup DSI_HSYNC_Polarity DSI HSYNC Polarity
  * @{
  */
#define DSI_HSYNC_ACTIVE_HIGH       0x00000000U
#define DSI_HSYNC_ACTIVE_LOW        DSI_LPCR_HSP
/**
  * @}
  */

/** @defgroup DSI_VSYNC_Active_Polarity DSI VSYNC Active Polarity
  * @{
  */
#define DSI_VSYNC_ACTIVE_HIGH       0x00000000U
#define DSI_VSYNC_ACTIVE_LOW        DSI_LPCR_VSP
/**
  * @}
  */

/** @defgroup DSI_DATA_ENABLE_Polarity DSI DATA ENABLE Polarity
  * @{
  */
#define DSI_DATA_ENABLE_ACTIVE_HIGH 0x00000000U
#define DSI_DATA_ENABLE_ACTIVE_LOW  DSI_LPCR_DEP
/**
  * @}
  */

/** @defgroup DSI_PLL_IDF DSI PLL IDF
  * @{
  */
#define DSI_PLL_IN_DIV1             0x00000001U
#define DSI_PLL_IN_DIV2             0x00000002U
#define DSI_PLL_IN_DIV3             0x00000003U
#define DSI_PLL_IN_DIV4             0x00000004U
#define DSI_PLL_IN_DIV5             0x00000005U
#define DSI_PLL_IN_DIV6             0x00000006U
#define DSI_PLL_IN_DIV7             0x00000007U
/**
  * @}
  */

/** @defgroup DSI_PLL_ODF DSI PLL ODF
  * @{
  */
#define DSI_PLL_OUT_DIV1            0x00000000U
#define DSI_PLL_OUT_DIV2            0x00000001U
#define DSI_PLL_OUT_DIV4            0x00000002U
#define DSI_PLL_OUT_DIV8            0x00000003U
/**
  * @}
  */

/** @defgroup DSI_Flags DSI Flags
  * @{
  */
#define DSI_FLAG_TE                 DSI_WISR_TEIF
#define DSI_FLAG_ER                 DSI_WISR_ERIF
#define DSI_FLAG_BUSY               DSI_WISR_BUSY
#define DSI_FLAG_PLLLS              DSI_WISR_PLLLS
#define DSI_FLAG_PLLL               DSI_WISR_PLLLIF
#define DSI_FLAG_PLLU               DSI_WISR_PLLUIF
#define DSI_FLAG_RRS                DSI_WISR_RRS
#define DSI_FLAG_RR                 DSI_WISR_RRIF
/**
  * @}
  */

/** @defgroup DSI_Interrupts DSI Interrupts
  * @{
  */
#define DSI_IT_TE                   DSI_WIER_TEIE
#define DSI_IT_ER                   DSI_WIER_ERIE
#define DSI_IT_PLLL                 DSI_WIER_PLLLIE
#define DSI_IT_PLLU                 DSI_WIER_PLLUIE
#define DSI_IT_RR                   DSI_WIER_RRIE
/**
  * @}
  */

/** @defgroup DSI_SHORT_WRITE_PKT_Data_Type DSI SHORT WRITE PKT Data Type
  * @{
  */
#define DSI_DCS_SHORT_PKT_WRITE_P0  0x00000005U /*!< DCS short write, no parameters      */
#define DSI_DCS_SHORT_PKT_WRITE_P1  0x00000015U /*!< DCS short write, one parameter      */
#define DSI_GEN_SHORT_PKT_WRITE_P0  0x00000003U /*!< Generic short write, no parameters  */
#define DSI_GEN_SHORT_PKT_WRITE_P1  0x00000013U /*!< Generic short write, one parameter  */
#define DSI_GEN_SHORT_PKT_WRITE_P2  0x00000023U /*!< Generic short write, two parameters */
/**
  * @}
  */

/** @defgroup DSI_LONG_WRITE_PKT_Data_Type DSI LONG WRITE PKT Data Type
  * @{
  */
#define DSI_DCS_LONG_PKT_WRITE      0x00000039U /*!< DCS long write     */
#define DSI_GEN_LONG_PKT_WRITE      0x00000029U /*!< Generic long write */
/**
  * @}
  */

/** @defgroup DSI_SHORT_READ_PKT_Data_Type DSI SHORT READ PKT Data Type
  * @{
  */
#define DSI_DCS_SHORT_PKT_READ      0x00000006U /*!< DCS short read                     */
#define DSI_GEN_SHORT_PKT_READ_P0   0x00000004U /*!< Generic short read, no parameters  */
#define DSI_GEN_SHORT_PKT_READ_P1   0x00000014U /*!< Generic short read, one parameter  */
#define DSI_GEN_SHORT_PKT_READ_P2   0x00000024U /*!< Generic short read, two parameters */
/**
  * @}
  */

/** @defgroup DSI_Error_Data_Type DSI Error Data Type
  * @{
  */
#define HAL_DSI_ERROR_NONE          0U
#define HAL_DSI_ERROR_ACK           0x00000001U /*!< acknowledge errors          */
#define HAL_DSI_ERROR_PHY           0x00000002U /*!< PHY related errors          */
#define HAL_DSI_ERROR_TX            0x00000004U /*!< transmission error          */
#define HAL_DSI_ERROR_RX            0x00000008U /*!< reception error             */
#define HAL_DSI_ERROR_ECC           0x00000010U /*!< ECC errors                  */
#define HAL_DSI_ERROR_CRC           0x00000020U /*!< CRC error                   */
#define HAL_DSI_ERROR_PSE           0x00000040U /*!< Packet Size error           */
#define HAL_DSI_ERROR_EOT           0x00000080U /*!< End Of Transmission error   */
#define HAL_DSI_ERROR_OVF           0x00000100U /*!< FIFO overflow error         */
#define HAL_DSI_ERROR_GEN           0x00000200U /*!< Generic FIFO related errors */
/**
  * @}
  */

/** @defgroup DSI_Lane_Group DSI Lane Group
  * @{
  */
#define DSI_CLOCK_LANE              0x00000000U
#define DSI_DATA_LANES              0x00000001U
/**
  * @}
  */

/** @defgroup DSI_Communication_Delay DSI Communication Delay
  * @{
  */
#define DSI_SLEW_RATE_HSTX          0x00000000U
#define DSI_SLEW_RATE_LPTX          0x00000001U
#define DSI_HS_DELAY                0x00000002U
/**
  * @}
  */

/** @defgroup DSI_CustomLane DSI CustomLane
  * @{
  */
#define DSI_SWAP_LANE_PINS          0x00000000U
#define DSI_INVERT_HS_SIGNAL        0x00000001U
/**
  * @}
  */

/** @defgroup DSI_Lane_Select DSI Lane Select
  * @{
  */
#define DSI_CLK_LANE                0x00000000U
#define DSI_DATA_LANE0              0x00000001U
#define DSI_DATA_LANE1              0x00000002U
/**
  * @}
  */

/** @defgroup DSI_PHY_Timing DSI PHY Timing
  * @{
  */
#define DSI_TCLK_POST               0x00000000U
#define DSI_TLPX_CLK                0x00000001U
#define DSI_THS_EXIT                0x00000002U
#define DSI_TLPX_DATA               0x00000003U
#define DSI_THS_ZERO                0x00000004U
#define DSI_THS_TRAIL               0x00000005U
#define DSI_THS_PREPARE             0x00000006U
#define DSI_TCLK_ZERO               0x00000007U
#define DSI_TCLK_PREPARE            0x00000008U
/**
  * @}
  */



/** @defgroup DSI_Clock_Bps DSI clock bps(DDR)
  * @brief
  * @{
  */
#define DSI_FREQ_96Mbps               96000000
#define DSI_FREQ_144Mbps              144000000
#define DSI_FREQ_168Mbps              168000000
#define DSI_FREQ_192Mbps              192000000
#define DSI_FREQ_240Mbps              240000000
#define DSI_FREQ_288Mbps              288000000
#define DSI_FREQ_336Mbps              336000000
#define DSI_FREQ_384Mbps              384000000
#define DSI_FREQ_480Mbps              480000000
#define DSI_FREQ_528Mbps              528000000
/**
  * @}
  */
/**
  * @}
  */


/* Exported macros -----------------------------------------------------------*/
/** @defgroup DSI_Exported_Macros DSI Exported Macros
  * @brief
  * @{
  */


/**
  * @brief  Enables the DSI host.
  * @param  \__HANDLE__  DSI handle
  * @retval None.
  */
#define __HAL_DSI_ENABLE(__HANDLE__) do { \
                                          __IO uint32_t tmpreg = 0x00U; \
                                          SET_BIT((__HANDLE__)->Instance->CTRL, DSI_HOST_CTRL_PU);\
                                          /* Delay after an DSI Host enabling */ \
                                          tmpreg = READ_BIT((__HANDLE__)->Instance->CTRL, DSI_HOST_CTRL_PU);\
                                          UNUSED(tmpreg); \
                                        }while(0U)

/**
  * @brief  Disables the DSI host.
  * @param  \__HANDLE__  DSI handle
  * @retval None.
  */
#define __HAL_DSI_DISABLE(__HANDLE__) do { \
                                          __IO uint32_t tmpreg = 0x00U; \
                                          CLEAR_BIT((__HANDLE__)->Instance->CTRL, DSI_HOST_CTRL_PU);\
                                          /* Delay after an DSI Host disabling */ \
                                          tmpreg = READ_BIT((__HANDLE__)->Instance->CTRL, DSI_HOST_CTRL_PU);\
                                          UNUSED(tmpreg); \
                                         }while(0U)

#define DSI_HOST_WRITE_FLAGS    (DSI_HOST_CMD_PKT_STAT_COM_CMD_EMPTY|DSI_HOST_CMD_PKT_STAT_COM_WRPLD_EMPTY|DSI_HOST_CMD_PKT_STAT_DBI_WRPLD_EMPTY|DSI_HOST_CMD_PKT_STAT_DBI_CMD_EMPTY)

#define __HAL_DSI_IS_WRITE_FINISH(__HANDLE__) (DSI_HOST_WRITE_FLAGS == ((__HANDLE__)->Instance->CMD_PKT_STAT & (DSI_HOST_WRITE_FLAGS)))
#define __HAL_DSI_IS_DLANE0_STOP(__HANDLE__)  (DSI_HOST_PHY_STAT_DLANE0_STOP == ((__HANDLE__)->Instance->PHY_STAT & (DSI_HOST_PHY_STAT_DLANE0_STOP_Msk)))
#define __HAL_DSI_IS_CKLANE_STOP(__HANDLE__)  (DSI_HOST_PHY_STAT_CKLANE_STOP == ((__HANDLE__)->Instance->PHY_STAT & (DSI_HOST_PHY_STAT_CKLANE_STOP_Msk)))
#define __HAL_DSI_IS_MASTER_STATE(__HANDLE__) (0 == ((__HANDLE__)->Instance->PHY_STAT & (DSI_HOST_PHY_STAT_DIR_Msk)))

#define __HAL_DSI_IS_DBI_INF_DELAY(__HANDLE__, DELAY) do { \
                                          CLEAR_BIT((__HANDLE__)->Instance->DBI_CFG1, DSI_HOST_DBI_CFG1_INTF_DLY_SEL_Msk);\
                                          SET_BIT((__HANDLE__)->Instance->DBI_CFG1, ((DELAY) << DSI_HOST_DBI_CFG1_INTF_DLY_SEL_Pos));\
                                         }while(0U)


/**
  * @brief  Enables the DSI PHY.
  * @param  \__HANDLE__  DSI handle
  * @retval None.
  */
#define __HAL_DSI_PHY_ENABLE(__HANDLE__) do { \
                                                 __IO uint32_t tmpreg = 0x00U; \
                                                 SET_BIT((__HANDLE__)->PHY_Instance->PHY_CFG, (DSI_PHY_PHY_CFG_ENABLE|DSI_PHY_PHY_CFG_RESETN));\
                                                 /* Delay after an DSI warpper enabling */ \
                                                 tmpreg = READ_BIT((__HANDLE__)->PHY_Instance->PHY_CFG, (DSI_PHY_PHY_CFG_ENABLE|DSI_PHY_PHY_CFG_RESETN));\
                                                 UNUSED(tmpreg); \
                                                }while(0U)

/**
  * @brief  Disable the DSI PHY.
  * @param  \__HANDLE__  DSI handle
  * @retval None.
  */
#define __HAL_DSI_PHY_DISABLE(__HANDLE__) do { \
                                                  __IO uint32_t tmpreg = 0x00U; \
                                                  CLEAR_BIT((__HANDLE__)->PHY_Instance->PHY_CFG, (DSI_PHY_PHY_CFG_ENABLE|DSI_PHY_PHY_CFG_RESETN));\
                                                  /* Delay after an DSI warpper disabling*/ \
                                                  tmpreg = READ_BIT((__HANDLE__)->PHY_Instance->PHY_CFG, (DSI_PHY_PHY_CFG_ENABLE|DSI_PHY_PHY_CFG_RESETN));\
                                                  UNUSED(tmpreg); \
                                                 }while(0U)


/**
  * @brief  Enables the DSI PLL.
  * @param  \__HANDLE__  DSI handle
  * @retval None.
  */
#define __HAL_DSI_PLL_ENABLE(__HANDLE__) do { \
                                             __IO uint32_t tmpreg = 0x00U; \
                                             SET_BIT((__HANDLE__)->PHY_Instance->DLL_CTRL, (DSI_PHY_DLL_CTRL_DLL0_LDO_EN));\
                                             HAL_Delay_us(5U);\
                                            SET_BIT((__HANDLE__)->PHY_Instance->DLL_CTRL, (DSI_PHY_DLL_CTRL_DLL0_OUT_RSTB|DSI_PHY_DLL_CTRL_DLL0_LOOP_EN));\
                                            HAL_Delay_us(5U);\
                                            SET_BIT((__HANDLE__)->PHY_Instance->DLL_CTRL, (DSI_PHY_DLL_CTRL_DLL0_OUT_EN));\
                                             /* Delay after an DSI PLL enabling */ \
                                             tmpreg = READ_BIT((__HANDLE__)->PHY_Instance->DLL_CTRL, (DSI_PHY_DLL_CTRL_DLL0_OUT_EN|DSI_PHY_DLL_CTRL_DLL0_OUT_RSTB|DSI_PHY_DLL_CTRL_DLL0_LOOP_EN|DSI_PHY_DLL_CTRL_DLL0_LDO_EN));\
                                             UNUSED(tmpreg); \
                                            }while(0U)

/**
  * @brief  Disables the DSI PLL.
  * @param  \__HANDLE__  DSI handle
  * @retval None.
  */
#define __HAL_DSI_PLL_DISABLE(__HANDLE__) do { \
                                              __IO uint32_t tmpreg = 0x00U; \
                                              CLEAR_BIT((__HANDLE__)->PHY_Instance->DLL_CTRL, (DSI_PHY_DLL_CTRL_DLL0_OUT_EN|DSI_PHY_DLL_CTRL_DLL0_OUT_RSTB|DSI_PHY_DLL_CTRL_DLL0_LOOP_EN|DSI_PHY_DLL_CTRL_DLL0_LDO_EN));\
                                              /* Delay after an DSI PLL disabling */ \
                                              tmpreg = READ_BIT((__HANDLE__)->PHY_Instance->DLL_CTRL, (DSI_PHY_DLL_CTRL_DLL0_OUT_EN|DSI_PHY_DLL_CTRL_DLL0_OUT_RSTB|DSI_PHY_DLL_CTRL_DLL0_LOOP_EN|DSI_PHY_DLL_CTRL_DLL0_LDO_EN));\
                                              UNUSED(tmpreg); \
                                             }while(0U)

/**
  * @brief  Enables the DSI analog power.
  * @param  \__HANDLE__  DSI handle
  * @retval None.
  */
#define __HAL_DSI_ANA_ENABLE(__HANDLE__) do { \
                                              __IO uint32_t tmpreg = 0x00U; \
                                              SET_BIT((__HANDLE__)->PHY_Instance->BIAS_CTRL, (DSI_PHY_BIAS_CTRL_PSW_EN|DSI_PHY_BIAS_CTRL_BIAS_EN|DSI_PHY_BIAS_CTRL_RSET_EN|DSI_PHY_BIAS_CTRL_BIAS_PROG_EN));\
                                              /* Delay after an DSI regulator enabling */ \
                                              tmpreg = READ_BIT((__HANDLE__)->PHY_Instance->BIAS_CTRL, (DSI_PHY_BIAS_CTRL_PSW_EN|DSI_PHY_BIAS_CTRL_BIAS_EN|DSI_PHY_BIAS_CTRL_RSET_EN|DSI_PHY_BIAS_CTRL_BIAS_PROG_EN));\
                                              UNUSED(tmpreg); \
                                            }while(0U)

/**
  * @brief  Disables the DSI analog power.
  * @param  \__HANDLE__  DSI handle
  * @retval None.
  */
#define __HAL_DSI_ANA_DISABLE(__HANDLE__) do { \
                                              __IO uint32_t tmpreg = 0x00U; \
                                              CLEAR_BIT((__HANDLE__)->PHY_Instance->BIAS_CTRL, (DSI_PHY_BIAS_CTRL_PSW_EN|DSI_PHY_BIAS_CTRL_BIAS_EN|DSI_PHY_BIAS_CTRL_RSET_EN|DSI_PHY_BIAS_CTRL_BIAS_PROG_EN));\
                                              /* Delay after an DSI regulator disabling */ \
                                              tmpreg = READ_BIT((__HANDLE__)->PHY_Instance->BIAS_CTRL, (DSI_PHY_BIAS_CTRL_PSW_EN|DSI_PHY_BIAS_CTRL_BIAS_EN|DSI_PHY_BIAS_CTRL_RSET_EN|DSI_PHY_BIAS_CTRL_BIAS_PROG_EN));\
                                              UNUSED(tmpreg); \
                                             }while(0U)
/**
  * @brief  DSI in video mode.
  * @param  \__HANDLE__  DSI handle
  * @retval None.
  */
#define HAL_DSI_IS_VIDEO_MODE(__HANDLE__) (0 == ((__HANDLE__)->Instance->MODE_CFG&DSI_HOST_MODE_CFG_CMD_VIDEO_SEL_Msk))

/**
  * @brief  Enable DSI video mode.
  * @param  \__HANDLE__  DSI handle
  * @retval None.
  */
#define HAL_DSI_VIDEO_MODE_ENABLE(__HANDLE__) do { \
                                              __IO uint32_t tmpreg = 0x00U; \
                                              CLEAR_BIT((__HANDLE__)->Instance->MODE_CFG, (DSI_HOST_MODE_CFG_CMD_VIDEO_SEL_Msk));\
                                              /* Delay after an DSI regulator enabling */ \
                                              tmpreg = READ_BIT((__HANDLE__)->Instance->MODE_CFG, (DSI_HOST_MODE_CFG_CMD_VIDEO_SEL_Msk));\
                                              UNUSED(tmpreg); \
                                            }while(0U)



#ifdef DSI_REMOVE_ME
/**
  * @brief  Get the DSI pending flags.
  * @param  \__HANDLE__  DSI handle.
  * @param  __FLAG__  Get the specified flag.
  *          This parameter can be any combination of the following values:
  *            @arg DSI_FLAG_TE   : Tearing Effect Interrupt Flag
  *            @arg DSI_FLAG_ER   : End of Refresh Interrupt Flag
  *            @arg DSI_FLAG_BUSY : Busy Flag
  *            @arg DSI_FLAG_PLLLS: PLL Lock Status
  *            @arg DSI_FLAG_PLLL : PLL Lock Interrupt Flag
  *            @arg DSI_FLAG_PLLU : PLL Unlock Interrupt Flag
  *            @arg DSI_FLAG_RRS  : Regulator Ready Flag
  *            @arg DSI_FLAG_RR   : Regulator Ready Interrupt Flag
  * @retval The state of FLAG (SET or RESET).
  */
#define __HAL_DSI_GET_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->Instance->WISR & (__FLAG__))

/**
  * @brief  Clears the DSI pending flags.
  * @param  \__HANDLE__  DSI handle.
  * @param  __FLAG__  specifies the flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg DSI_FLAG_TE   : Tearing Effect Interrupt Flag
  *            @arg DSI_FLAG_ER   : End of Refresh Interrupt Flag
  *            @arg DSI_FLAG_PLLL : PLL Lock Interrupt Flag
  *            @arg DSI_FLAG_PLLU : PLL Unlock Interrupt Flag
  *            @arg DSI_FLAG_RR   : Regulator Ready Interrupt Flag
  * @retval None
  */
#define __HAL_DSI_CLEAR_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->Instance->WIFCR = (__FLAG__))

/**
  * @brief  Enables the specified DSI interrupts.
  * @param  \__HANDLE__  DSI handle.
  * @param __INTERRUPT__  specifies the DSI interrupt sources to be enabled.
  *          This parameter can be any combination of the following values:
  *            @arg DSI_IT_TE  : Tearing Effect Interrupt
  *            @arg DSI_IT_ER  : End of Refresh Interrupt
  *            @arg DSI_IT_PLLL: PLL Lock Interrupt
  *            @arg DSI_IT_PLLU: PLL Unlock Interrupt
  *            @arg DSI_IT_RR  : Regulator Ready Interrupt
  * @retval None
  */
#define __HAL_DSI_ENABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->WIER |= (__INTERRUPT__))

/**
  * @brief  Disables the specified DSI interrupts.
  * @param  \__HANDLE__  DSI handle
  * @param __INTERRUPT__  specifies the DSI interrupt sources to be disabled.
  *          This parameter can be any combination of the following values:
  *            @arg DSI_IT_TE  : Tearing Effect Interrupt
  *            @arg DSI_IT_ER  : End of Refresh Interrupt
  *            @arg DSI_IT_PLLL: PLL Lock Interrupt
  *            @arg DSI_IT_PLLU: PLL Unlock Interrupt
  *            @arg DSI_IT_RR  : Regulator Ready Interrupt
  * @retval None
  */
#define __HAL_DSI_DISABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->WIER &= ~(__INTERRUPT__))

/**
  * @brief  Checks whether the specified DSI interrupt source is enabled or not.
  * @param  \__HANDLE__  DSI handle
  * @param  __INTERRUPT__  specifies the DSI interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg DSI_IT_TE  : Tearing Effect Interrupt
  *            @arg DSI_IT_ER  : End of Refresh Interrupt
  *            @arg DSI_IT_PLLL: PLL Lock Interrupt
  *            @arg DSI_IT_PLLU: PLL Unlock Interrupt
  *            @arg DSI_IT_RR  : Regulator Ready Interrupt
  * @retval The state of INTERRUPT (SET or RESET).
  */
#define __HAL_DSI_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->WIER & (__INTERRUPT__))



#else /* DSI_REMOVE_ME */

#define __HAL_DSI_GET_FLAG(__HANDLE__, __FLAG__) (SET)
#define __HAL_DSI_CLEAR_FLAG(__HANDLE__, __FLAG__)
#define __HAL_DSI_ENABLE_IT(__HANDLE__, __INTERRUPT__)
#define __HAL_DSI_DISABLE_IT(__HANDLE__, __INTERRUPT__)
#define __HAL_DSI_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)


#endif /* DSI_REMOVE_ME */


/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @defgroup DSI_Exported_Functions DSI Exported Functions
  * @{
  */

/** @defgroup DSI_Group1 Initialization and Configuration functions
 *  @brief   Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
                ##### Initialization and Configuration functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize and configure the DSI
      (+) De-initialize the DSI

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the DSI according to the specified
  *         parameters in the DSI_InitTypeDef and create the associated handle.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_Init(DSI_HandleTypeDef *hdsi);


/**
  * @brief  De-initializes the DSI peripheral registers to their default reset
  *         values.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_DeInit(DSI_HandleTypeDef *hdsi);

/**
  * @brief  Initializes the DSI MSP.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval None
  */
void HAL_DSI_MspInit(DSI_HandleTypeDef *hdsi);

/**
  * @brief  De-initializes the DSI MSP.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval None
  */
void HAL_DSI_MspDeInit(DSI_HandleTypeDef *hdsi);

/**
  * @}
  */

/** @defgroup DSI_Group2 IO operation functions
 *  @brief    IO operation functions
 *
@verbatim
 ===============================================================================
                      #####  IO operation functions  #####
 ===============================================================================
    [..]  This section provides function allowing to:
      (+) Handle DSI interrupt request

@endverbatim
  * @{
  */
/**
  * @brief  Handles DSI interrupt request.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
void HAL_DSI_IRQHandler(DSI_HandleTypeDef *hdsi);

/**
  * @brief  Tearing Effect DSI callback.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval None
  */
void HAL_DSI_TearingEffectCallback(DSI_HandleTypeDef *hdsi);

/**
  * @brief  End of Refresh DSI callback.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval None
  */
void HAL_DSI_EndOfRefreshCallback(DSI_HandleTypeDef *hdsi);

/**
  * @brief  Operation Error DSI callback.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval None
  */
void HAL_DSI_ErrorCallback(DSI_HandleTypeDef *hdsi);

/**
  * @}
  */


/** @defgroup DSI_Group3 Peripheral Control functions
 *  @brief    Peripheral Control functions
 *
@verbatim
 ===============================================================================
                    ##### Peripheral Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure the Generic interface read-back Virtual Channel ID
      (+) Select video mode and configure the corresponding parameters
      (+) Configure command transmission mode: High-speed or Low-power
      (+) Configure the flow control
      (+) Configure the DSI PHY timer
      (+) Configure the DSI HOST timeout
      (+) Configure the DSI HOST timeout
      (+) Start/Stop the DSI module
      (+) Refresh the display in command mode
      (+) Controls the display color mode in Video mode
      (+) Control the display shutdown in Video mode
      (+) write short DCS or short Generic command
      (+) write long DCS or long Generic command
      (+) Read command (DCS or generic)
      (+) Enter/Exit the Ultra Low Power Mode on data only (D-PHY PLL running)
      (+) Enter/Exit the Ultra Low Power Mode on data only and clock (D-PHY PLL turned off)
      (+) Start/Stop test pattern generation
      (+) Slew-Rate And Delay Tuning
      (+) Low-Power Reception Filter Tuning
      (+) Activate an additional current path on all lanes to meet the SDDTx parameter
      (+) Custom lane pins configuration
      (+) Set custom timing for the PHY
      (+) Force the Clock/Data Lane in TX Stop Mode
      (+) Force LP Receiver in Low-Power Mode
      (+) Force Data Lanes in RX Mode after a BTA
      (+) Enable a pull-down on the lanes to prevent from floating states when unused
      (+) Switch off the contention detection on data lanes

@endverbatim
  * @{
  */

/**
  * @brief  Configure the Generic interface read-back Virtual Channel ID.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  VirtualChannelID  Virtual channel ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_SetGenericVCID(DSI_HandleTypeDef *hdsi, uint32_t VirtualChannelID);
HAL_StatusTypeDef HAL_DSI_ConfigVideoMode(DSI_HandleTypeDef *hdsi, DSI_VidCfgTypeDef *VidCfg);
/**
  * @brief  Select adapted command mode and configure the corresponding parameters
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  CmdCfg  pointer to a DSI_CmdCfgTypeDef structure that contains
  *                 the DSI command mode configuration parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ConfigAdaptedCommandMode(DSI_HandleTypeDef *hdsi, DSI_CmdCfgTypeDef *CmdCfg);
/**
  * @brief  Configure command transmission mode: High-speed or Low-power
  *         and enable/disable acknowledge request after packet transmission
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  LPCmd  pointer to a DSI_LPCmdTypeDef structure that contains
  *                the DSI command transmission mode configuration parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ConfigCommand(DSI_HandleTypeDef *hdsi, DSI_LPCmdTypeDef *LPCmd);
/**
  * @brief  Configure the flow control parameters
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  FlowControl  flow control feature(s) to be enabled.
  *                      This parameter can be any combination of @ref DSI_FlowControl.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ConfigFlowControl(DSI_HandleTypeDef *hdsi, uint32_t FlowControl);
/**
  * @brief  Configure the DSI PHY timer parameters
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  PhyTimers  DSI_PHY_TimerTypeDef structure that contains
  *                    the DSI PHY timing parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ConfigPhyTimer(DSI_HandleTypeDef *hdsi, DSI_PHY_TimerTypeDef *PhyTimers);
/**
  * @brief  Configure the DSI HOST timeout parameters
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  HostTimeouts  DSI_HOST_TimeoutTypeDef structure that contains
  *                       the DSI host timeout parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ConfigHostTimeouts(DSI_HandleTypeDef *hdsi, DSI_HOST_TimeoutTypeDef *HostTimeouts);

/**
  * @brief  Configure the DSI PHY frequency.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  freq  DSI frequency.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ConfigPhyFreq(DSI_HandleTypeDef *hdsi, uint32_t freq);

/**
 * @brief Set patial write buffer bytes
 * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
 *               the configuration information for the DSI.
 * @param write_size - framebuffer bytes
 * @return
 */
HAL_StatusTypeDef HAL_DSI_SetPaticalSend(DSI_HandleTypeDef *hdsi, uint32_t write_size);

#ifndef SF32LB55X
/**
 * @brief Set direct write buffer bytes
 * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
 *               the configuration information for the DSI.
 * @param write_size - framebuffer bytes
 * @return
 */
HAL_StatusTypeDef HAL_DSI_DirectSend(DSI_HandleTypeDef *hdsi,     uint32_t write_size);
#endif

/**
  * @brief  Start the DSI module
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_Start(DSI_HandleTypeDef *hdsi);

/**
  * @brief  Bypass all lane output on bus
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  bypass  1: bypass enabled, 0: bypass disabled.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_bypass_output(DSI_HandleTypeDef *hdsi, bool bypass);


/**
  * @brief  Stop the DSI module
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_Stop(DSI_HandleTypeDef *hdsi);

/**
  * @brief  write short DCS or short Generic command
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  ChannelID  Virtual channel ID.
  * @param  Mode  DSI short packet data type.
  *               This parameter can be any value of @ref DSI_SHORT_WRITE_PKT_Data_Type.
  * @param  Param1  DSC command or first generic parameter.
  *                 This parameter can be any value of @ref DSI_DCS_Command or a
  *                 generic command code.
  * @param  Param2  DSC parameter or second generic parameter.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ShortWrite(DSI_HandleTypeDef *hdsi,
                                     uint32_t ChannelID,
                                     uint32_t Mode,
                                     uint32_t Param1,
                                     uint32_t Param2);
/**
  * @brief  write long DCS or long Generic command
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  ChannelID  Virtual channel ID.
  * @param  Mode  DSI long packet data type.
  *               This parameter can be any value of @ref DSI_LONG_WRITE_PKT_Data_Type.
  * @param  NbParams  Number of parameters.
  * @param  Param1  DSC command or first generic parameter.
  *                 This parameter can be any value of @ref DSI_DCS_Command or a
  *                 generic command code
  * @param  ParametersTable  Pointer to parameter values table.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_LongWrite(DSI_HandleTypeDef *hdsi,
                                    uint32_t ChannelID,
                                    uint32_t Mode,
                                    uint32_t NbParams,
                                    uint32_t Param1,
                                    uint8_t *ParametersTable);
/**
  * @brief  Read command (DCS or generic)
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  ChannelNbr  Virtual channel ID
  * @param  Array pointer to a buffer to store the payload of a read back operation.
  * @param  Size  Data size to be read (in byte).
  * @param  Mode  DSI read packet data type.
  *               This parameter can be any value of @ref DSI_SHORT_READ_PKT_Data_Type.
  * @param  DCSCmd  DCS get/read command.
  * @param  ParametersTable  Pointer to parameter values table.
  * @retval HAL status
*/
HAL_StatusTypeDef HAL_DSI_Read(DSI_HandleTypeDef *hdsi,
                               uint32_t ChannelNbr,
                               uint8_t *Array,
                               uint32_t Size,
                               uint32_t Mode,
                               uint32_t DCSCmd,
                               uint8_t *ParametersTable);

/**
  * @brief  Enter the ULPM (Ultra Low Power Mode) with the D-PHY PLL running
  *         (only data lanes are in ULPM)
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_EnterULPMData(DSI_HandleTypeDef *hdsi);

/**
  * @brief  Exit the ULPM (Ultra Low Power Mode) with the D-PHY PLL running
  *         (only data lanes are in ULPM)
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ExitULPMData(DSI_HandleTypeDef *hdsi);
/**
  * @brief  Enter the ULPM (Ultra Low Power Mode) with the D-PHY PLL turned off
  *         (both data and clock lanes are in ULPM)
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_EnterULPM(DSI_HandleTypeDef *hdsi);
/**
  * @brief  Exit the ULPM (Ultra Low Power Mode) with the D-PHY PLL turned off
  *         (both data and clock lanes are in ULPM)
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ExitULPM(DSI_HandleTypeDef *hdsi);

/**
  * @brief  Enable the error monitor flags
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  ActiveErrors  indicates which error interrupts will be enabled.
  *                      This parameter can be any combination of @ref DSI_Error_Data_Type.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ConfigErrorMonitor(DSI_HandleTypeDef *hdsi, uint32_t ActiveErrors);


/**
  * @brief  Clock lane Enter the HighSpeed mode and close clock lane auto control
  *
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_EnterCKLaneHSM(DSI_HandleTypeDef *hdsi);

/**
  * @brief  Resume clock lane auto control by hdsi->Init.AutomaticClockLaneControl
  *
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ExitCKLaneHSM(DSI_HandleTypeDef *hdsi);


/**
  * @brief  Check DSI is busy
  *
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status, TRUE if during BTA, or reading, writting
  */
uint8_t HAL_DSI_IsBusy(DSI_HandleTypeDef *hdsi);
/**
  * @}
  */

/** @defgroup DSI_Group4 Peripheral State and Errors functions
 *  @brief    Peripheral State and Errors functions
 *
@verbatim
 ===============================================================================
                  ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Check the DSI state.
      (+) Get error code.

@endverbatim
  * @{
  */
/**
  * @brief  Return the DSI state
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL state
  */

HAL_DSI_StateTypeDef HAL_DSI_GetState(DSI_HandleTypeDef *hdsi);
/**
  * @brief  Return the DSI error code
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval DSI Error Code
  */
uint32_t HAL_DSI_GetError(DSI_HandleTypeDef *hdsi);
/**
  * @}
  */



void HAL_DSI_ForceEnableAck(void);
uint8_t HAL_DSI_ISForceEnableAck(void);

/**
  * @}DSI_Exported_Functions
  */


/* Private types -------------------------------------------------------------*/
/** @defgroup DSI_Private_Types DSI Private Types
  * @{
  */

/**
  * @}
  */

/* Private defines -----------------------------------------------------------*/
/** @defgroup DSI_Private_Defines DSI Private Defines
  * @{
  */

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup DSI_Private_Variables DSI Private Variables
  * @{
  */

/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup DSI_Private_Constants DSI Private Constants
  * @{
  */
#define DSI_MAX_RETURN_PKT_SIZE (0x00000037U) /*!< Maximum return packet configuration */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup DSI_Private_Macros DSI Private Macros
  * @{
  */
#define IS_DSI_PLL_NDIV(NDIV)                       ((10U <= (NDIV)) && ((NDIV) <= 125U))
#define IS_DSI_PLL_IDF(IDF)                         (((IDF) == DSI_PLL_IN_DIV1) || \
                                                     ((IDF) == DSI_PLL_IN_DIV2) || \
                                                     ((IDF) == DSI_PLL_IN_DIV3) || \
                                                     ((IDF) == DSI_PLL_IN_DIV4) || \
                                                     ((IDF) == DSI_PLL_IN_DIV5) || \
                                                     ((IDF) == DSI_PLL_IN_DIV6) || \
                                                     ((IDF) == DSI_PLL_IN_DIV7))
#define IS_DSI_PLL_ODF(ODF)                         (((ODF) == DSI_PLL_OUT_DIV1) || \
                                                     ((ODF) == DSI_PLL_OUT_DIV2) || \
                                                     ((ODF) == DSI_PLL_OUT_DIV4) || \
                                                     ((ODF) == DSI_PLL_OUT_DIV8))
#define IS_DSI_AUTO_CLKLANE_CONTROL(AutoClkLane)    (((AutoClkLane) == DSI_AUTO_CLK_LANE_CTRL_DISABLE) || ((AutoClkLane) == DSI_AUTO_CLK_LANE_CTRL_ENABLE))
#define IS_DSI_NUMBER_OF_LANES(NumberOfLanes)       (((NumberOfLanes) == DSI_ONE_DATA_LANE) || ((NumberOfLanes) == DSI_TWO_DATA_LANES))
#define IS_DSI_FLOW_CONTROL(FlowControl)            (((FlowControl) | DSI_FLOW_CONTROL_ALL) == DSI_FLOW_CONTROL_ALL)
#define IS_DSI_COLOR_CODING(ColorCoding)            ((ColorCoding) <= 5U)
#define IS_DSI_LOOSELY_PACKED(LooselyPacked)        (((LooselyPacked) == DSI_LOOSELY_PACKED_ENABLE) || ((LooselyPacked) == DSI_LOOSELY_PACKED_DISABLE))
#define IS_DSI_DE_POLARITY(DataEnable)              (((DataEnable) == DSI_DATA_ENABLE_ACTIVE_HIGH) || ((DataEnable) == DSI_DATA_ENABLE_ACTIVE_LOW))
#define IS_DSI_VSYNC_POLARITY(VSYNC)                (((VSYNC) == DSI_VSYNC_ACTIVE_HIGH) || ((VSYNC) == DSI_VSYNC_ACTIVE_LOW))
#define IS_DSI_HSYNC_POLARITY(HSYNC)                (((HSYNC) == DSI_HSYNC_ACTIVE_HIGH) || ((HSYNC) == DSI_HSYNC_ACTIVE_LOW))
#define IS_DSI_VIDEO_MODE_TYPE(VideoModeType)       (((VideoModeType) == DSI_VID_MODE_NB_PULSES) || \
                                                     ((VideoModeType) == DSI_VID_MODE_NB_EVENTS) || \
                                                     ((VideoModeType) == DSI_VID_MODE_BURST))
#define IS_DSI_COLOR_MODE(ColorMode)                (((ColorMode) == DSI_COLOR_MODE_FULL) || ((ColorMode) == DSI_COLOR_MODE_EIGHT))
#define IS_DSI_SHUT_DOWN(ShutDown)                  (((ShutDown) == DSI_DISPLAY_ON) || ((ShutDown) == DSI_DISPLAY_OFF))
#define IS_DSI_LP_COMMAND(LPCommand)                (((LPCommand) == DSI_LP_COMMAND_DISABLE) || ((LPCommand) == DSI_LP_COMMAND_ENABLE))
#define IS_DSI_LP_HFP(LPHFP)                        (((LPHFP) == DSI_LP_HFP_DISABLE) || ((LPHFP) == DSI_LP_HFP_ENABLE))
#define IS_DSI_LP_HBP(LPHBP)                        (((LPHBP) == DSI_LP_HBP_DISABLE) || ((LPHBP) == DSI_LP_HBP_ENABLE))
#define IS_DSI_LP_VACTIVE(LPVActive)                (((LPVActive) == DSI_LP_VACT_DISABLE) || ((LPVActive) == DSI_LP_VACT_ENABLE))
#define IS_DSI_LP_VFP(LPVFP)                        (((LPVFP) == DSI_LP_VFP_DISABLE) || ((LPVFP) == DSI_LP_VFP_ENABLE))
#define IS_DSI_LP_VBP(LPVBP)                        (((LPVBP) == DSI_LP_VBP_DISABLE) || ((LPVBP) == DSI_LP_VBP_ENABLE))
#define IS_DSI_LP_VSYNC(LPVSYNC)                    (((LPVSYNC) == DSI_LP_VSYNC_DISABLE) || ((LPVSYNC) == DSI_LP_VSYNC_ENABLE))
#define IS_DSI_FBTAA(FrameBTAAcknowledge)           (((FrameBTAAcknowledge) == DSI_FBTAA_DISABLE) || ((FrameBTAAcknowledge) == DSI_FBTAA_ENABLE))
#define IS_DSI_TE_SOURCE(TESource)                  (((TESource) == DSI_TE_DSILINK) || ((TESource) == DSI_TE_EXTERNAL))
#define IS_DSI_TE_POLARITY(TEPolarity)              (((TEPolarity) == DSI_TE_RISING_EDGE) || ((TEPolarity) == DSI_TE_FALLING_EDGE))
#define IS_DSI_AUTOMATIC_REFRESH(AutomaticRefresh)  (((AutomaticRefresh) == DSI_AR_DISABLE) || ((AutomaticRefresh) == DSI_AR_ENABLE))
#define IS_DSI_VS_POLARITY(VSPolarity)              (((VSPolarity) == DSI_VSYNC_FALLING) || ((VSPolarity) == DSI_VSYNC_RISING))
#define IS_DSI_TE_ACK_REQUEST(TEAcknowledgeRequest) (((TEAcknowledgeRequest) == DSI_TE_ACKNOWLEDGE_DISABLE) || ((TEAcknowledgeRequest) == DSI_TE_ACKNOWLEDGE_ENABLE))
#define IS_DSI_ACK_REQUEST(AcknowledgeRequest)      (((AcknowledgeRequest) == DSI_ACKNOWLEDGE_DISABLE) || ((AcknowledgeRequest) == DSI_ACKNOWLEDGE_ENABLE))
#define IS_DSI_LP_GSW0P(LP_GSW0P)                   (((LP_GSW0P) == DSI_LP_GSW0P_DISABLE) || ((LP_GSW0P) == DSI_LP_GSW0P_ENABLE))
#define IS_DSI_LP_GSW1P(LP_GSW1P)                   (((LP_GSW1P) == DSI_LP_GSW1P_DISABLE) || ((LP_GSW1P) == DSI_LP_GSW1P_ENABLE))
#define IS_DSI_LP_GSW2P(LP_GSW2P)                   (((LP_GSW2P) == DSI_LP_GSW2P_DISABLE) || ((LP_GSW2P) == DSI_LP_GSW2P_ENABLE))
#define IS_DSI_LP_GSR0P(LP_GSR0P)                   (((LP_GSR0P) == DSI_LP_GSR0P_DISABLE) || ((LP_GSR0P) == DSI_LP_GSR0P_ENABLE))
#define IS_DSI_LP_GSR1P(LP_GSR1P)                   (((LP_GSR1P) == DSI_LP_GSR1P_DISABLE) || ((LP_GSR1P) == DSI_LP_GSR1P_ENABLE))
#define IS_DSI_LP_GSR2P(LP_GSR2P)                   (((LP_GSR2P) == DSI_LP_GSR2P_DISABLE) || ((LP_GSR2P) == DSI_LP_GSR2P_ENABLE))
#define IS_DSI_LP_GLW(LP_GLW)                       (((LP_GLW) == DSI_LP_GLW_DISABLE) || ((LP_GLW) == DSI_LP_GLW_ENABLE))
#define IS_DSI_LP_DSW0P(LP_DSW0P)                   (((LP_DSW0P) == DSI_LP_DSW0P_DISABLE) || ((LP_DSW0P) == DSI_LP_DSW0P_ENABLE))
#define IS_DSI_LP_DSW1P(LP_DSW1P)                   (((LP_DSW1P) == DSI_LP_DSW1P_DISABLE) || ((LP_DSW1P) == DSI_LP_DSW1P_ENABLE))
#define IS_DSI_LP_DSR0P(LP_DSR0P)                   (((LP_DSR0P) == DSI_LP_DSR0P_DISABLE) || ((LP_DSR0P) == DSI_LP_DSR0P_ENABLE))
#define IS_DSI_LP_DLW(LP_DLW)                       (((LP_DLW) == DSI_LP_DLW_DISABLE) || ((LP_DLW) == DSI_LP_DLW_ENABLE))
#define IS_DSI_LP_MRDP(LP_MRDP)                     (((LP_MRDP) == DSI_LP_MRDP_DISABLE) || ((LP_MRDP) == DSI_LP_MRDP_ENABLE))
#define IS_DSI_SHORT_WRITE_PACKET_TYPE(MODE)        (((MODE) == DSI_DCS_SHORT_PKT_WRITE_P0) || \
                                                     ((MODE) == DSI_DCS_SHORT_PKT_WRITE_P1) || \
                                                     ((MODE) == DSI_GEN_SHORT_PKT_WRITE_P0) || \
                                                     ((MODE) == DSI_GEN_SHORT_PKT_WRITE_P1) || \
                                                     ((MODE) == DSI_GEN_SHORT_PKT_WRITE_P2))
#define IS_DSI_LONG_WRITE_PACKET_TYPE(MODE)         (((MODE) == DSI_DCS_LONG_PKT_WRITE) || \
                                                     ((MODE) == DSI_GEN_LONG_PKT_WRITE))
#define IS_DSI_READ_PACKET_TYPE(MODE)               (((MODE) == DSI_DCS_SHORT_PKT_READ) || \
                                                     ((MODE) == DSI_GEN_SHORT_PKT_READ_P0) || \
                                                     ((MODE) == DSI_GEN_SHORT_PKT_READ_P1) || \
                                                     ((MODE) == DSI_GEN_SHORT_PKT_READ_P2))
#define IS_DSI_COMMUNICATION_DELAY(CommDelay)       (((CommDelay) == DSI_SLEW_RATE_HSTX) || ((CommDelay) == DSI_SLEW_RATE_LPTX) || ((CommDelay) == DSI_HS_DELAY))
#define IS_DSI_LANE_GROUP(Lane)                     (((Lane) == DSI_CLOCK_LANE) || ((Lane) == DSI_DATA_LANES))
#define IS_DSI_CUSTOM_LANE(CustomLane)              (((CustomLane) == DSI_SWAP_LANE_PINS) || ((CustomLane) == DSI_INVERT_HS_SIGNAL))
#define IS_DSI_LANE(Lane)                           (((Lane) == DSI_CLOCK_LANE) || ((Lane) == DSI_DATA_LANE0) || ((Lane) == DSI_DATA_LANE1))
#define IS_DSI_PHY_TIMING(Timing)                   (((Timing) == DSI_TCLK_POST   ) || \
                                                     ((Timing) == DSI_TLPX_CLK    ) || \
                                                     ((Timing) == DSI_THS_EXIT    ) || \
                                                     ((Timing) == DSI_TLPX_DATA   ) || \
                                                     ((Timing) == DSI_THS_ZERO    ) || \
                                                     ((Timing) == DSI_THS_TRAIL   ) || \
                                                     ((Timing) == DSI_THS_PREPARE ) || \
                                                     ((Timing) == DSI_TCLK_ZERO   ) || \
                                                     ((Timing) == DSI_TCLK_PREPARE))

/**
  * @}DSI_Private_Macros
  */

/* Private functions prototypes ----------------------------------------------*/
/** @defgroup DSI_Private_Functions_Prototypes DSI Private Functions Prototypes
  * @{
  */

/**
  * @}DSI_Private_Functions_Prototypes
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup DSI_Private_Functions DSI Private Functions
  * @{
  */

/**
  * @}DSI_Private_Functions
  */

/**
  * @} DSI
  */

/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /* BF0_HAL_DSI_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

