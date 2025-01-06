/**
  ******************************************************************************
  * @file   bf0_hal_sd_ex.h
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

#ifndef __BF0_HAL_SDHCI_EX_H__
#define __BF0_HAL_SDHCI_EX_H__

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"
#include "bf0_hal_sdhci.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup SDEX SD Extertion
  * @{
  */

/** @defgroup SDEX_SDMMC SDEX SDMMC
  * @{
  */

/* Exported types --------------------------------------------------------*/
/** @defgroup SDEX_SDMMC_Define SDEX SDMMC Define
  * @{
  */

#define SDMMC_ERROR_NONE                     0x00000000U   /*!< No error                                                      */
#define SDMMC_ERROR_CMD_CRC_FAIL             0x00000001U   /*!< Command response received (but CRC check failed)              */
#define SDMMC_ERROR_DATA_CRC_FAIL            0x00000002U   /*!< Data block sent/received (CRC check failed)                   */
#define SDMMC_ERROR_CMD_RSP_TIMEOUT          0x00000004U   /*!< Command response timeout                                      */
#define SDMMC_ERROR_DATA_TIMEOUT             0x00000008U   /*!< Data timeout                                                  */
#define SDMMC_ERROR_TX_UNDERRUN              0x00000010U   /*!< Transmit FIFO underrun                                        */
#define SDMMC_ERROR_RX_OVERRUN               0x00000020U   /*!< Receive FIFO overrun                                          */
#define SDMMC_ERROR_ADDR_MISALIGNED          0x00000040U   /*!< Misaligned address                                            */
#define SDMMC_ERROR_BLOCK_LEN_ERR            0x00000080U   /*!< Transferred block length is not allowed for the card or the 
                                                                 number of transferred bytes does not match the block length   */
#define SDMMC_ERROR_ERASE_SEQ_ERR            0x00000100U   /*!< An error in the sequence of erase command occurs              */
#define SDMMC_ERROR_BAD_ERASE_PARAM          0x00000200U   /*!< An invalid selection for erase groups                         */
#define SDMMC_ERROR_WRITE_PROT_VIOLATION     0x00000400U   /*!< Attempt to program a write protect block                      */
#define SDMMC_ERROR_LOCK_UNLOCK_FAILED       0x00000800U   /*!< Sequence or password error has been detected in unlock 
                                                                command or if there was an attempt to access a locked card    */
#define SDMMC_ERROR_COM_CRC_FAILED           0x00001000U   /*!< CRC check of the previous command failed                      */
#define SDMMC_ERROR_ILLEGAL_CMD              0x00002000U   /*!< Command is not legal for the card state                       */
#define SDMMC_ERROR_CARD_ECC_FAILED          0x00004000U   /*!< Card internal ECC was applied but failed to correct the data  */
#define SDMMC_ERROR_CC_ERR                   0x00008000U   /*!< Internal card controller error                                */
#define SDMMC_ERROR_GENERAL_UNKNOWN_ERR      0x00010000U   /*!< General or unknown error                                      */
#define SDMMC_ERROR_STREAM_READ_UNDERRUN     0x00020000U   /*!< The card could not sustain data reading in stream rmode       */
#define SDMMC_ERROR_STREAM_WRITE_OVERRUN     0x00040000U   /*!< The card could not sustain data programming in stream mode    */
#define SDMMC_ERROR_CID_CSD_OVERWRITE        0x00080000U   /*!< CID/CSD overwrite error                                       */
#define SDMMC_ERROR_WP_ERASE_SKIP            0x00100000U   /*!< Only partial address space was erased                         */
#define SDMMC_ERROR_CARD_ECC_DISABLED        0x00200000U   /*!< Command has been executed without using internal ECC          */
#define SDMMC_ERROR_ERASE_RESET              0x00400000U   /*!< Erase sequence was cleared before executing because an out 
                                                                of erase sequence command was received                        */
#define SDMMC_ERROR_AKE_SEQ_ERR              0x00800000U   /*!< Error in sequence of authentication                           */
#define SDMMC_ERROR_INVALID_VOLTRANGE        0x01000000U   /*!< Error in case of invalid voltage range                        */
#define SDMMC_ERROR_ADDR_OUT_OF_RANGE        0x02000000U   /*!< Error when addressed block is out of range                    */
#define SDMMC_ERROR_REQUEST_NOT_APPLICABLE   0x04000000U   /*!< Error when command request is not applicable                  */
#define SDMMC_ERROR_INVALID_PARAMETER        0x08000000U   /*!< the used parameter is not valid                               */
#define SDMMC_ERROR_UNSUPPORTED_FEATURE      0x10000000U   /*!< Error when feature is not insupported                         */
#define SDMMC_ERROR_BUSY                     0x20000000U   /*!< Error when transfer process is busy                           */
#define SDMMC_ERROR_DMA                      0x40000000U   /*!< Error while DMA transfer                                      */
#define SDMMC_ERROR_TIMEOUT                  0x80000000U   /*!< Timeout error                                                 */

/**
  * @brief SDMMC Commands Index
  */
#define SDMMC_CMD_GO_IDLE_STATE                       ((uint8_t)0)   /*!< Resets the SD memory card.                                                               */
#define SDMMC_CMD_SEND_OP_COND                        ((uint8_t)1)   /*!< Sends host capacity support information and activates the card's initialization process. */
#define SDMMC_CMD_ALL_SEND_CID                        ((uint8_t)2)   /*!< Asks any card connected to the host to send the CID numbers on the CMD line.             */
#define SDMMC_CMD_SET_REL_ADDR                        ((uint8_t)3)   /*!< Asks the card to publish a new relative address (RCA).                                   */
#define SDMMC_CMD_SET_DSR                             ((uint8_t)4)   /*!< Programs the DSR of all cards.                                                           */
#define SDMMC_CMD_SDMMC_SEN_OP_COND                   ((uint8_t)5)   /*!< Sends host capacity support information (HCS) and asks the accessed card to send its 
                                                                       operating condition register (OCR) content in the response on the CMD line.                  */
#define SDMMC_CMD_HS_SWITCH                           ((uint8_t)6)   /*!< Checks switchable function (mode 0) and switch card function (mode 1).                   */
#define SDMMC_CMD_SEL_DESEL_CARD                      ((uint8_t)7)   /*!< Selects the card by its own relative address and gets deselected by any other address    */
#define SDMMC_CMD_HS_SEND_EXT_CSD                     ((uint8_t)8)   /*!< Sends SD Memory Card interface condition, which includes host supply voltage information 
                                                                       and asks the card whether card supports voltage.                                             */
#define SDMMC_CMD_SEND_CSD                            ((uint8_t)9)   /*!< Addressed card sends its card specific data (CSD) on the CMD line.                       */
#define SDMMC_CMD_SEND_CID                            ((uint8_t)10)  /*!< Addressed card sends its card identification (CID) on the CMD line.                      */
#define SDMMC_CMD_READ_DAT_UNTIL_STOP                 ((uint8_t)11)  /*!< SD card doesn't support it.                                                              */
#define SDMMC_CMD_STOP_TRANSMISSION                   ((uint8_t)12)  /*!< Forces the card to stop transmission.                                                    */
#define SDMMC_CMD_SEND_STATUS                         ((uint8_t)13)  /*!< Addressed card sends its status register.                                                */
#define SDMMC_CMD_HS_BUSTEST_READ                     ((uint8_t)14)  /*!< Reserved                                                                                 */
#define SDMMC_CMD_GO_INACTIVE_STATE                   ((uint8_t)15)  /*!< Sends an addressed card into the inactive state.                                         */
#define SDMMC_CMD_SET_BLOCKLEN                        ((uint8_t)16)  /*!< Sets the block length (in bytes for SDSC) for all following block commands 
                                                                           (read, write, lock). Default block length is fixed to 512 Bytes. Not effective 
                                                                           for SDHS and SDXC.                                                                       */
#define SDMMC_CMD_READ_SINGLE_BLOCK                   ((uint8_t)17)  /*!< Reads single block of size selected by SET_BLOCKLEN in case of SDSC, and a block of 
                                                                           fixed 512 bytes in case of SDHC and SDXC.                                                */
#define SDMMC_CMD_READ_MULT_BLOCK                     ((uint8_t)18)  /*!< Continuously transfers data blocks from card to host until interrupted by 
                                                                           STOP_TRANSMISSION command.                                                               */
#define SDMMC_CMD_HS_BUSTEST_WRITE                    ((uint8_t)19)  /*!< 64 bytes tuning pattern is sent for SDR50 and SDR104.                                    */
#define SDMMC_CMD_WRITE_DAT_UNTIL_STOP                ((uint8_t)20)  /*!< Speed class control command.                                                             */
#define SDMMC_CMD_SET_BLOCK_COUNT                     ((uint8_t)23)  /*!< Specify block count for CMD18 and CMD25.                                                 */
#define SDMMC_CMD_WRITE_SINGLE_BLOCK                  ((uint8_t)24)  /*!< Writes single block of size selected by SET_BLOCKLEN in case of SDSC, and a block of 
                                                                           fixed 512 bytes in case of SDHC and SDXC.                                                */
#define SDMMC_CMD_WRITE_MULT_BLOCK                    ((uint8_t)25)  /*!< Continuously writes blocks of data until a STOP_TRANSMISSION follows.                    */
#define SDMMC_CMD_PROG_CID                            ((uint8_t)26)  /*!< Reserved for manufacturers.                                                              */
#define SDMMC_CMD_PROG_CSD                            ((uint8_t)27)  /*!< Programming of the programmable bits of the CSD.                                         */
#define SDMMC_CMD_SET_WRITE_PROT                      ((uint8_t)28)  /*!< Sets the write protection bit of the addressed group.                                    */
#define SDMMC_CMD_CLR_WRITE_PROT                      ((uint8_t)29)  /*!< Clears the write protection bit of the addressed group.                                  */
#define SDMMC_CMD_SEND_WRITE_PROT                     ((uint8_t)30)  /*!< Asks the card to send the status of the write protection bits.                           */
#define SDMMC_CMD_SD_ERASE_GRP_START                  ((uint8_t)32)  /*!< Sets the address of the first write block to be erased. (For SD card only).              */
#define SDMMC_CMD_SD_ERASE_GRP_END                    ((uint8_t)33)  /*!< Sets the address of the last write block of the continuous range to be erased.           */
#define SDMMC_CMD_ERASE_GRP_START                     ((uint8_t)35)  /*!< Sets the address of the first write block to be erased. Reserved for each command 
                                                                           system set by switch function command (CMD6).                                            */
#define SDMMC_CMD_ERASE_GRP_END                       ((uint8_t)36)  /*!< Sets the address of the last write block of the continuous range to be erased. 
                                                                           Reserved for each command system set by switch function command (CMD6).                  */
#define SDMMC_CMD_ERASE                               ((uint8_t)38)  /*!< Reserved for SD security applications.                                                   */
#define SDMMC_CMD_FAST_IO                             ((uint8_t)39)  /*!< SD card doesn't support it (Reserved).                                                   */
#define SDMMC_CMD_GO_IRQ_STATE                        ((uint8_t)40)  /*!< SD card doesn't support it (Reserved).                                                   */
#define SDMMC_CMD_LOCK_UNLOCK                         ((uint8_t)42)  /*!< Sets/resets the password or lock/unlock the card. The size of the data block is set by 
                                                                           the SET_BLOCK_LEN command.                                                               */
#define SDMMC_CMD_APP_CMD                             ((uint8_t)55)  /*!< Indicates to the card that the next command is an application specific command rather 
                                                                           than a standard command.                                                                 */
#define SDMMC_CMD_GEN_CMD                             ((uint8_t)56)  /*!< Used either to transfer a data block to the card or to get a data block from the card 
                                                                           for general purpose/application specific commands.                                       */
#define SDMMC_CMD_NO_CMD                              ((uint8_t)64)  /*!< No command                                                                               */

/**
  * @brief Following commands are SD Card Specific commands.
  *        SDMMC_APP_CMD should be sent before sending these commands.
  */
#define SDMMC_CMD_APP_SD_SET_BUSWIDTH                 ((uint8_t)6)   /*!< (ACMD6) Defines the data bus width to be used for data transfer. The allowed data bus 
                                                                            widths are given in SCR register.                                                       */
#define SDMMC_CMD_SD_APP_STATUS                       ((uint8_t)13)  /*!< (ACMD13) Sends the SD status.                                                            */
#define SDMMC_CMD_SD_APP_SEND_NUM_WRITE_BLOCKS        ((uint8_t)22)  /*!< (ACMD22) Sends the number of the written (without errors) write blocks. Responds with 
                                                                           32bit+CRC data block.                                                                    */
#define SDMMC_CMD_SD_APP_OP_COND                      ((uint8_t)41)  /*!< (ACMD41) Sends host capacity support information (HCS) and asks the accessed card to 
                                                                           send its operating condition register (OCR) content in the response on the CMD line.     */
#define SDMMC_CMD_SD_APP_SET_CLR_CARD_DETECT          ((uint8_t)42)  /*!< (ACMD42) Connect/Disconnect the 50 KOhm pull-up resistor on CD/DAT3 (pin 1) of the card  */
#define SDMMC_CMD_SD_APP_SEND_SCR                     ((uint8_t)51)  /*!< Reads the SD Configuration Register (SCR).                                               */
#define SDMMC_CMD_SDMMC_RW_DIRECT                     ((uint8_t)52)  /*!< For SD I/O card only, reserved for security specification.                               */
#define SDMMC_CMD_SDMMC_RW_EXTENDED                   ((uint8_t)53)  /*!< For SD I/O card only, reserved for security specification.                               */

/**
  * @brief Following commands are SD Card Specific security commands.
  *        SDMMC_CMD_APP_CMD should be sent before sending these commands.
  */
#define SDMMC_CMD_SD_APP_GET_MKB                      ((uint8_t)43)
#define SDMMC_CMD_SD_APP_GET_MID                      ((uint8_t)44)
#define SDMMC_CMD_SD_APP_SET_CER_RN1                  ((uint8_t)45)
#define SDMMC_CMD_SD_APP_GET_CER_RN2                  ((uint8_t)46)
#define SDMMC_CMD_SD_APP_SET_CER_RES2                 ((uint8_t)47)
#define SDMMC_CMD_SD_APP_GET_CER_RES1                 ((uint8_t)48)
#define SDMMC_CMD_SD_APP_SECURE_READ_MULTIPLE_BLOCK   ((uint8_t)18)
#define SDMMC_CMD_SD_APP_SECURE_WRITE_MULTIPLE_BLOCK  ((uint8_t)25)
#define SDMMC_CMD_SD_APP_SECURE_ERASE                 ((uint8_t)38)
#define SDMMC_CMD_SD_APP_CHANGE_SECURE_AREA           ((uint8_t)49)
#define SDMMC_CMD_SD_APP_SECURE_WRITE_MKB             ((uint8_t)48)


/**
  * @brief  Masks for errors Card Status R1 (OCR Register)
  */
#define SDMMC_OCR_ADDR_OUT_OF_RANGE        0x80000000U
#define SDMMC_OCR_ADDR_MISALIGNED          0x40000000U
#define SDMMC_OCR_BLOCK_LEN_ERR            0x20000000U
#define SDMMC_OCR_ERASE_SEQ_ERR            0x10000000U
#define SDMMC_OCR_BAD_ERASE_PARAM          0x08000000U
#define SDMMC_OCR_WRITE_PROT_VIOLATION     0x04000000U
#define SDMMC_OCR_LOCK_UNLOCK_FAILED       0x01000000U
#define SDMMC_OCR_COM_CRC_FAILED           0x00800000U
#define SDMMC_OCR_ILLEGAL_CMD              0x00400000U
#define SDMMC_OCR_CARD_ECC_FAILED          0x00200000U
#define SDMMC_OCR_CC_ERROR                 0x00100000U
#define SDMMC_OCR_GENERAL_UNKNOWN_ERROR    0x00080000U
#define SDMMC_OCR_STREAM_READ_UNDERRUN     0x00040000U
#define SDMMC_OCR_STREAM_WRITE_OVERRUN     0x00020000U
#define SDMMC_OCR_CID_CSD_OVERWRITE        0x00010000U
#define SDMMC_OCR_WP_ERASE_SKIP            0x00008000U
#define SDMMC_OCR_CARD_ECC_DISABLED        0x00004000U
#define SDMMC_OCR_ERASE_RESET              0x00002000U
#define SDMMC_OCR_AKE_SEQ_ERROR            0x00000008U
#define SDMMC_OCR_ERRORBITS                0xFDFFE008U

/**
  * @brief  Masks for R6 Response
  */
#define SDMMC_R6_GENERAL_UNKNOWN_ERROR     0x00002000U
#define SDMMC_R6_ILLEGAL_CMD               0x00004000U
#define SDMMC_R6_COM_CRC_FAILED            0x00008000U

#define SDMMC_VOLTAGE_WINDOW_SD            0x80100000U
#define SDMMC_HIGH_CAPACITY                0x40000000U
#define SDMMC_STD_CAPACITY                 0x00000000U
#define SDMMC_CHECK_PATTERN                0x000001AAU

#define SDMMC_MAX_TRIAL               0x0000FFFFU

#define SDMMC_ALLZERO                      0x00000000U

#define SDMMC_WIDE_BUS_SUPPORT             0x00040000U
#define SDMMC_SINGLE_BUS_SUPPORT           0x00010000U
#define SDMMC_CARD_LOCKED                  0x02000000U

#define SDMMC_DATATIMEOUT                  0xFFFFFFFFU

#define SDMMC_0TO7BITS                     0x000000FFU
#define SDMMC_8TO15BITS                    0x0000FF00U
#define SDMMC_16TO23BITS                   0x00FF0000U
#define SDMMC_24TO31BITS                   0xFF000000U
#define SDMMC_MAX_DATA_LENGTH              0x01FFFFFFU

/**
  * @}
  */

/** @defgroup SDEX_Exported_Types SDEX Exported Types
  * @{
  */

/**
  * @brief  SDMMC Configuration Structure definition
  */
typedef struct
{
    uint32_t ClockEdge;            /*!< Specifies the clock transition on which the bit capture is made.          */

    uint32_t ClockBypass;          /*!< Specifies whether the SDMMC Clock divider bypass is
                                      enabled or disabled.           */

    uint32_t ClockPowerSave;       /*!< Specifies whether SDMMC Clock output is enabled or
                                      disabled when the bus is idle.         */

    uint32_t BusWide;              /*!< Specifies the SDMMC bus width.                  */

    uint32_t HardwareFlowControl;  /*!< Specifies whether the SDMMC hardware flow control is enabled or disabled.     */

    uint32_t ClockDiv;             /*!< Specifies the clock frequency of the SDMMC controller.
                                      This parameter can be a value between Min_Data = 0 and Max_Data = 255 */

} SDIO_InitTypeDef;

/**
  * @brief  SDMMC Data Control structure
  */
typedef struct
{
    uint32_t DataTimeOut;         /*!< Specifies the data timeout period in card bus clock periods.  */

    uint32_t DataLength;          /*!< Specifies the number of data bytes to be transferred.         */

    uint32_t DataBlockSize;       /*!< Specifies the data block size for block transfer    */

    uint32_t TransferDir;         /*!< Specifies the data transfer direction, whether the transfer
                                     is a read or write */

    uint32_t TransferMode;        /*!< Specifies whether data transfer is in stream or block mode      */

    uint32_t DMAMode;             /*!< Specifies dma transfer mode       */
} SDIO_DataInitTypeDef;

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup SDEX_Exported_Functions SDEX Exported Functions
  * @{
  */

/* SDMMC Cards mode management functions */
HAL_StatusTypeDef SDIO_SetSDMMCReadWaitMode(SDHCI_HandleTypeDef *SDIOx, uint32_t SDIO_ReadWaitMode);

/* SDMMC Commands management functions */
uint32_t SDMMC_CmdBlockLength(SDHCI_HandleTypeDef *SDIOx, uint32_t BlockSize);
uint32_t SDMMC_CmdReadSingleBlock(SDHCI_HandleTypeDef *SDIOx, uint32_t ReadAdd);
uint32_t SDMMC_CmdReadMultiBlock(SDHCI_HandleTypeDef *SDIOx, uint32_t ReadAdd);
uint32_t SDMMC_CmdWriteSingleBlock(SDHCI_HandleTypeDef *SDIOx, uint32_t WriteAdd);
uint32_t SDMMC_CmdWriteMultiBlock(SDHCI_HandleTypeDef *SDIOx, uint32_t WriteAdd);
uint32_t SDMMC_CmdSDEraseStartAdd(SDHCI_HandleTypeDef *SDIOx, uint32_t StartAdd);
uint32_t SDMMC_CmdSDEraseEndAdd(SDHCI_HandleTypeDef *SDIOx, uint32_t EndAdd);
uint32_t SDMMC_CmdErase(SDHCI_HandleTypeDef *SDIOx);
uint32_t SDMMC_CmdStopTransfer(SDHCI_HandleTypeDef *SDIOx);
uint32_t SDMMC_CmdSelDesel(SDHCI_HandleTypeDef *SDIOx, uint64_t Addr);
uint32_t SDMMC_CmdGoIdleState(SDHCI_HandleTypeDef *SDIOx);
uint32_t SDMMC_CmdOperCond(SDHCI_HandleTypeDef *SDIOx);
uint32_t SDMMC_CmdAppCommand(SDHCI_HandleTypeDef *SDIOx, uint32_t Argument);
uint32_t SDMMC_CmdAppOperCommand(SDHCI_HandleTypeDef *SDIOx, uint32_t SdType);
uint32_t SDMMC_CmdBusWidth(SDHCI_HandleTypeDef *SDIOx, uint32_t BusWidth);
uint32_t SDMMC_CmdSendSCR(SDHCI_HandleTypeDef *SDIOx);
uint32_t SDMMC_CmdSendCID(SDHCI_HandleTypeDef *SDIOx);
uint32_t SDMMC_CmdSendCSD(SDHCI_HandleTypeDef *SDIOx, uint32_t Argument);
uint32_t SDMMC_CmdSetRelAdd(SDHCI_HandleTypeDef *SDIOx, uint16_t *pRCA);
uint32_t SDMMC_CmdSendStatus(SDHCI_HandleTypeDef *SDIOx, uint32_t Argument);
uint32_t SDMMC_CmdStatusRegister(SDHCI_HandleTypeDef *SDIOx);

uint32_t SDMMC_CmdOpCondition(SDHCI_HandleTypeDef *SDIOx, uint32_t Argument);
uint32_t SDMMC_CmdSwitch(SDHCI_HandleTypeDef *SDIOx, uint32_t Argument);
uint32_t SDMMC_CmdEraseStartAdd(SDHCI_HandleTypeDef *SDIOx, uint32_t StartAdd);
uint32_t SDMMC_CmdEraseEndAdd(SDHCI_HandleTypeDef *SDIOx, uint32_t EndAdd);

/**
  * @}
  */

/**
  * @}
  */

#if 1

/** @defgroup SDEX_SDCARD SDEX SDCARD
  * @{
  */

/* Exported types --------------------------------------------------------*/
/** @defgroup SDEX_SDCARD_Define SDEX SDCARD Define
  * @{
  */

#define BLOCKSIZE   512U /*!< Block size is 512 bytes */

/** @defgroup SD_Exported_Constansts_Group1 SD Error status enumeration Structure definition
  * @{
  */
#define HAL_SD_ERROR_NONE                     SDMMC_ERROR_NONE                    /*!< No error                                                      */
#define HAL_SD_ERROR_CMD_CRC_FAIL             SDMMC_ERROR_CMD_CRC_FAIL            /*!< Command response received (but CRC check failed)              */
#define HAL_SD_ERROR_DATA_CRC_FAIL            SDMMC_ERROR_DATA_CRC_FAIL           /*!< Data block sent/received (CRC check failed)                   */
#define HAL_SD_ERROR_CMD_RSP_TIMEOUT          SDMMC_ERROR_CMD_RSP_TIMEOUT         /*!< Command response timeout                                      */
#define HAL_SD_ERROR_DATA_TIMEOUT             SDMMC_ERROR_DATA_TIMEOUT            /*!< Data timeout                                                  */
#define HAL_SD_ERROR_TX_UNDERRUN              SDMMC_ERROR_TX_UNDERRUN             /*!< Transmit FIFO underrun                                        */
#define HAL_SD_ERROR_RX_OVERRUN               SDMMC_ERROR_RX_OVERRUN              /*!< Receive FIFO overrun                                          */
#define HAL_SD_ERROR_ADDR_MISALIGNED          SDMMC_ERROR_ADDR_MISALIGNED         /*!< Misaligned address                                            */
#define HAL_SD_ERROR_BLOCK_LEN_ERR            SDMMC_ERROR_BLOCK_LEN_ERR           /*!< Transferred block length is not allowed for the card or the 
                                                                                       number of transferred bytes does not match the block length   */
#define HAL_SD_ERROR_ERASE_SEQ_ERR            SDMMC_ERROR_ERASE_SEQ_ERR           /*!< An error in the sequence of erase command occurs              */
#define HAL_SD_ERROR_BAD_ERASE_PARAM          SDMMC_ERROR_BAD_ERASE_PARAM         /*!< An invalid selection for erase groups                         */
#define HAL_SD_ERROR_WRITE_PROT_VIOLATION     SDMMC_ERROR_WRITE_PROT_VIOLATION    /*!< Attempt to program a write protect block                      */
#define HAL_SD_ERROR_LOCK_UNLOCK_FAILED       SDMMC_ERROR_LOCK_UNLOCK_FAILED      /*!< Sequence or password error has been detected in unlock 
                                                                                       command or if there was an attempt to access a locked card    */
#define HAL_SD_ERROR_COM_CRC_FAILED           SDMMC_ERROR_COM_CRC_FAILED          /*!< CRC check of the previous command failed                      */
#define HAL_SD_ERROR_ILLEGAL_CMD              SDMMC_ERROR_ILLEGAL_CMD             /*!< Command is not legal for the card state                       */
#define HAL_SD_ERROR_CARD_ECC_FAILED          SDMMC_ERROR_CARD_ECC_FAILED         /*!< Card internal ECC was applied but failed to correct the data  */
#define HAL_SD_ERROR_CC_ERR                   SDMMC_ERROR_CC_ERR                  /*!< Internal card controller error                                */
#define HAL_SD_ERROR_GENERAL_UNKNOWN_ERR      SDMMC_ERROR_GENERAL_UNKNOWN_ERR     /*!< General or unknown error                                      */
#define HAL_SD_ERROR_STREAM_READ_UNDERRUN     SDMMC_ERROR_STREAM_READ_UNDERRUN    /*!< The card could not sustain data reading in stream rmode       */
#define HAL_SD_ERROR_STREAM_WRITE_OVERRUN     SDMMC_ERROR_STREAM_WRITE_OVERRUN    /*!< The card could not sustain data programming in stream mode    */
#define HAL_SD_ERROR_CID_CSD_OVERWRITE        SDMMC_ERROR_CID_CSD_OVERWRITE       /*!< CID/CSD overwrite error                                       */
#define HAL_SD_ERROR_WP_ERASE_SKIP            SDMMC_ERROR_WP_ERASE_SKIP           /*!< Only partial address space was erased                         */
#define HAL_SD_ERROR_CARD_ECC_DISABLED        SDMMC_ERROR_CARD_ECC_DISABLED       /*!< Command has been executed without using internal ECC          */
#define HAL_SD_ERROR_ERASE_RESET              SDMMC_ERROR_ERASE_RESET             /*!< Erase sequence was cleared before executing because an out 
                                                                                       of erase sequence command was received                        */
#define HAL_SD_ERROR_AKE_SEQ_ERR              SDMMC_ERROR_AKE_SEQ_ERR             /*!< Error in sequence of authentication                           */
#define HAL_SD_ERROR_INVALID_VOLTRANGE        SDMMC_ERROR_INVALID_VOLTRANGE       /*!< Error in case of invalid voltage range                        */
#define HAL_SD_ERROR_ADDR_OUT_OF_RANGE        SDMMC_ERROR_ADDR_OUT_OF_RANGE       /*!< Error when addressed block is out of range                    */
#define HAL_SD_ERROR_REQUEST_NOT_APPLICABLE   SDMMC_ERROR_REQUEST_NOT_APPLICABLE  /*!< Error when command request is not applicable                  */
#define HAL_SD_ERROR_PARAM                    SDMMC_ERROR_INVALID_PARAMETER       /*!< the used parameter is not valid                               */
#define HAL_SD_ERROR_UNSUPPORTED_FEATURE      SDMMC_ERROR_UNSUPPORTED_FEATURE     /*!< Error when feature is not insupported                         */
#define HAL_SD_ERROR_BUSY                     SDMMC_ERROR_BUSY                    /*!< Error when transfer process is busy                           */
#define HAL_SD_ERROR_DMA                      SDMMC_ERROR_DMA                     /*!< Error while DMA transfer                                      */
#define HAL_SD_ERROR_TIMEOUT                  SDMMC_ERROR_TIMEOUT                 /*!< Timeout error                                                 */

/**
  * @}
  */

/** @defgroup SD_Exported_Constansts_Group2 SD context enumeration
  * @{
  */
#define   SD_CONTEXT_NONE                 0x00000000U  /*!< None                             */
#define   SD_CONTEXT_READ_SINGLE_BLOCK    0x00000001U  /*!< Read single block operation      */
#define   SD_CONTEXT_READ_MULTIPLE_BLOCK  0x00000002U  /*!< Read multiple blocks operation   */
#define   SD_CONTEXT_WRITE_SINGLE_BLOCK   0x00000010U  /*!< Write single block operation     */
#define   SD_CONTEXT_WRITE_MULTIPLE_BLOCK 0x00000020U  /*!< Write multiple blocks operation  */
#define   SD_CONTEXT_IT                   0x00000008U  /*!< Process in Interrupt mode        */
#define   SD_CONTEXT_DMA                  0x00000080U  /*!< Process in DMA mode              */

/**
  * @}
  */

/** @defgroup SD_Exported_Constansts_Group3 SD Supported Memory Cards
  * @{
  */
#define CARD_SDSC                  0x00000000U
#define CARD_SDHC_SDXC             0x00000001U
#define CARD_SECURED               0x00000003U

/**
  * @}
  */

/** @defgroup SD_Exported_Constansts_Group4 SD Supported Version
  * @{
  */
#define CARD_V1_X                  0x00000000U
#define CARD_V2_X                  0x00000001U
/**
  * @}
  */

/**
  * @}
  */


/** @defgroup SD_Exported_Types_Group1 SD State enumeration structure
  * @{
  */
typedef enum
{
    HAL_SD_STATE_RESET                  = 0x00000000U,  /*!< SD not yet initialized or disabled  */
    HAL_SD_STATE_READY                  = 0x00000001U,  /*!< SD initialized and ready for use    */
    HAL_SD_STATE_TIMEOUT                = 0x00000002U,  /*!< SD Timeout state                    */
    HAL_SD_STATE_BUSY                   = 0x00000003U,  /*!< SD process ongoing                  */
    HAL_SD_STATE_PROGRAMMING            = 0x00000004U,  /*!< SD Programming State                */
    HAL_SD_STATE_RECEIVING              = 0x00000005U,  /*!< SD Receinving State                 */
    HAL_SD_STATE_TRANSFER               = 0x00000006U,  /*!< SD Transfert State                  */
    HAL_SD_STATE_ERROR                  = 0x0000000FU   /*!< SD is in error state                */
} HAL_SD_StateTypeDef;
/**
  * @}
  */

/** @defgroup SD_Exported_Types_Group2 SD Card State enumeration structure
  * @{
  */
typedef enum
{
    HAL_SD_CARD_READY                  = 0x00000001U,  /*!< Card state is ready                     */
    HAL_SD_CARD_IDENTIFICATION         = 0x00000002U,  /*!< Card is in identification state         */
    HAL_SD_CARD_STANDBY                = 0x00000003U,  /*!< Card is in standby state                */
    HAL_SD_CARD_TRANSFER               = 0x00000004U,  /*!< Card is in transfer state               */
    HAL_SD_CARD_SENDING                = 0x00000005U,  /*!< Card is sending an operation            */
    HAL_SD_CARD_RECEIVING              = 0x00000006U,  /*!< Card is receiving operation information */
    HAL_SD_CARD_PROGRAMMING            = 0x00000007U,  /*!< Card is in programming state            */
    HAL_SD_CARD_DISCONNECTED           = 0x00000008U,  /*!< Card is disconnected                    */
    HAL_SD_CARD_ERROR                  = 0x000000FFU   /*!< Card response Error                     */
} HAL_SD_CardStateTypeDef;
/**
  * @}
  */

/** @defgroup SD_Exported_Types_Group3 SD handle structure
  * @{
  */

/**
  * @brief  SD Card Information Structure definition
  */
typedef struct
{
    uint32_t CardType;                     /*!< Specifies the card Type                         */

    uint32_t CardVersion;                  /*!< Specifies the card version                      */

    uint32_t Class;                        /*!< Specifies the class of the card class           */

    uint32_t RelCardAdd;                   /*!< Specifies the Relative Card Address             */

    uint32_t BlockNbr;                     /*!< Specifies the Card Capacity in blocks           */

    uint32_t BlockSize;                    /*!< Specifies one block size in bytes               */

    uint32_t LogBlockNbr;                  /*!< Specifies the Card logical Capacity in blocks   */

    uint32_t LogBlockSize;                 /*!< Specifies logical block size in bytes           */

} HAL_SD_CardInfoTypeDef;

/**
  * @brief  SD handle Structure definition
  */
typedef struct
{
    SDHCI_HandleTypeDef                   *Instance;        /*!< SD registers base address           */

    SDIO_InitTypeDef               Init;             /*!< SD required parameters              */

    HAL_LockTypeDef              Lock;             /*!< SD locking object                   */

    uint32_t                     *pTxBuffPtr;      /*!< Pointer to SD Tx transfer Buffer    */

    uint32_t                     TxXferSize;       /*!< SD Tx Transfer size                 */

    uint32_t                     *pRxBuffPtr;      /*!< Pointer to SD Rx transfer Buffer    */

    uint32_t                     RxXferSize;       /*!< SD Rx Transfer size                 */

    __IO uint32_t                Context;          /*!< SD transfer context                 */

    __IO uint32_t     State;            /*!< SD card State                       */

    __IO uint32_t                ErrorCode;        /*!< SD Card Error codes                 */

    DMA_HandleTypeDef            *hdmarx;          /*!< SD Rx DMA handle parameters         */

    DMA_HandleTypeDef            *hdmatx;          /*!< SD Tx DMA handle parameters         */

    HAL_SD_CardInfoTypeDef       SdCard;           /*!< SD Card information                 */

    uint32_t                     CSD[4];           /*!< SD card specific data table         */

    uint32_t                     CID[4];           /*!< SD card identification number table */

} SD_HandleTypeDef;


/**
  * @}
  */


/** @defgroup SD_Exported_Types_Group4 Card Specific Data: CSD Register
  * @{
  */
typedef struct
{
    __IO uint8_t  CSDStruct;            /*!< CSD structure                         */
    __IO uint8_t  SysSpecVersion;       /*!< System specification version          */
    __IO uint8_t  Reserved1;            /*!< Reserved                              */
    __IO uint8_t  TAAC;                 /*!< Data read access time 1               */
    __IO uint8_t  NSAC;                 /*!< Data read access time 2 in CLK cycles */
    __IO uint8_t  MaxBusClkFrec;        /*!< Max. bus clock frequency              */
    __IO uint16_t CardComdClasses;      /*!< Card command classes                  */
    __IO uint8_t  RdBlockLen;           /*!< Max. read data block length           */
    __IO uint8_t  PartBlockRead;        /*!< Partial blocks for read allowed       */
    __IO uint8_t  WrBlockMisalign;      /*!< Write block misalignment              */
    __IO uint8_t  RdBlockMisalign;      /*!< Read block misalignment               */
    __IO uint8_t  DSRImpl;              /*!< DSR implemented                       */
    __IO uint8_t  Reserved2;            /*!< Reserved                              */
    __IO uint32_t DeviceSize;           /*!< Device Size                           */
    __IO uint8_t  MaxRdCurrentVDDMin;   /*!< Max. read current @ VDD min           */
    __IO uint8_t  MaxRdCurrentVDDMax;   /*!< Max. read current @ VDD max           */
    __IO uint8_t  MaxWrCurrentVDDMin;   /*!< Max. write current @ VDD min          */
    __IO uint8_t  MaxWrCurrentVDDMax;   /*!< Max. write current @ VDD max          */
    __IO uint8_t  DeviceSizeMul;        /*!< Device size multiplier                */
    __IO uint8_t  EraseGrSize;          /*!< Erase group size                      */
    __IO uint8_t  EraseGrMul;           /*!< Erase group size multiplier           */
    __IO uint8_t  WrProtectGrSize;      /*!< Write protect group size              */
    __IO uint8_t  WrProtectGrEnable;    /*!< Write protect group enable            */
    __IO uint8_t  ManDeflECC;           /*!< Manufacturer default ECC              */
    __IO uint8_t  WrSpeedFact;          /*!< Write speed factor                    */
    __IO uint8_t  MaxWrBlockLen;        /*!< Max. write data block length          */
    __IO uint8_t  WriteBlockPaPartial;  /*!< Partial blocks for write allowed      */
    __IO uint8_t  Reserved3;            /*!< Reserved                              */
    __IO uint8_t  ContentProtectAppli;  /*!< Content protection application        */
    __IO uint8_t  FileFormatGrouop;     /*!< File format group                     */
    __IO uint8_t  CopyFlag;             /*!< Copy flag (OTP)                       */
    __IO uint8_t  PermWrProtect;        /*!< Permanent write protection            */
    __IO uint8_t  TempWrProtect;        /*!< Temporary write protection            */
    __IO uint8_t  FileFormat;           /*!< File format                           */
    __IO uint8_t  ECC;                  /*!< ECC code                              */
    __IO uint8_t  CSD_CRC;              /*!< CSD CRC                               */
    __IO uint8_t  Reserved4;            /*!< Always 1                              */

} HAL_SD_CardCSDTypeDef;
/**
  * @}
  */

/** @defgroup SD_Exported_Types_Group5 Card Identification Data: CID Register
  * @{
  */
typedef struct
{
    __IO uint8_t  ManufacturerID;  /*!< Manufacturer ID       */
    __IO uint16_t OEM_AppliID;     /*!< OEM/Application ID    */
    __IO uint32_t ProdName1;       /*!< Product Name part1    */
    __IO uint8_t  ProdName2;       /*!< Product Name part2    */
    __IO uint8_t  ProdRev;         /*!< Product Revision      */
    __IO uint32_t ProdSN;          /*!< Product Serial Number */
    __IO uint8_t  Reserved1;       /*!< Reserved1             */
    __IO uint16_t ManufactDate;    /*!< Manufacturing Date    */
    __IO uint8_t  CID_CRC;         /*!< CID CRC               */
    __IO uint8_t  Reserved2;       /*!< Always 1              */

} HAL_SD_CardCIDTypeDef;
/**
  * @}
  */

/** @defgroup SD_Exported_Types_Group6 SD Card Status returned by ACMD13
  * @{
  */
typedef struct
{
    __IO uint8_t  DataBusWidth;           /*!< Shows the currently defined data bus width                 */
    __IO uint8_t  SecuredMode;            /*!< Card is in secured mode of operation                       */
    __IO uint16_t CardType;               /*!< Carries information about card type                        */
    __IO uint32_t ProtectedAreaSize;      /*!< Carries information about the capacity of protected area   */
    __IO uint8_t  SpeedClass;             /*!< Carries information about the speed class of the card      */
    __IO uint8_t  PerformanceMove;        /*!< Carries information about the card's performance move      */
    __IO uint8_t  AllocationUnitSize;     /*!< Carries information about the card's allocation unit size  */
    __IO uint16_t EraseSize;              /*!< Determines the number of AUs to be erased in one operation */
    __IO uint8_t  EraseTimeout;           /*!< Determines the timeout for any number of AU erase          */
    __IO uint8_t  EraseOffset;            /*!< Carries information about the erase offset                 */

} HAL_SD_CardStatusTypeDef;
/**
  * @}
  */

/** @defgroup SD_Exported_Functions_Group1 Initialization and de-initialization functions
* @{
*/
HAL_StatusTypeDef HAL_SD_Init(SD_HandleTypeDef *hsd);
HAL_StatusTypeDef HAL_SD_InitCard(SD_HandleTypeDef *hsd);
HAL_StatusTypeDef HAL_SD_DeInit(SD_HandleTypeDef *hsd);
void HAL_SD_MspInit(SD_HandleTypeDef *hsd);
void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd);
/**
* @}
*/

/** @defgroup SD_Exported_Functions_Group2 Input and Output operation functions
* @{
*/
/* Blocking mode: Polling */
HAL_StatusTypeDef HAL_SD_ReadBlocks(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout);
HAL_StatusTypeDef HAL_SD_WriteBlocks(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout);
HAL_StatusTypeDef HAL_SD_Erase(SD_HandleTypeDef *hsd, uint32_t BlockStartAdd, uint32_t BlockEndAdd);
/* Non-Blocking mode: IT */
HAL_StatusTypeDef HAL_SD_ReadBlocks_IT(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);
HAL_StatusTypeDef HAL_SD_WriteBlocks_IT(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);
/* Non-Blocking mode: DMA */
HAL_StatusTypeDef HAL_SD_ReadBlocks_DMA(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);
HAL_StatusTypeDef HAL_SD_WriteBlocks_DMA(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);

void HAL_SD_IRQHandler(SD_HandleTypeDef *hsd);

/* Callback in non blocking modes (DMA) */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd);
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd);
void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd);
void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd);
/**
* @}
*/

/** @defgroup SD_Exported_Functions_Group3 Peripheral Control functions
* @{
*/
HAL_StatusTypeDef HAL_SD_ConfigWideBusOperation(SD_HandleTypeDef *hsd, uint32_t WideMode);
/**
* @}
*/

/** @defgroup SD_Exported_Functions_Group4 SD card related functions
* @{
*/
HAL_StatusTypeDef       HAL_SD_SendSDStatus(SD_HandleTypeDef *hsd, uint32_t *pSDstatus);
HAL_SD_CardStateTypeDef HAL_SD_GetCardState(SD_HandleTypeDef *hsd);
HAL_StatusTypeDef       HAL_SD_GetCardCID(SD_HandleTypeDef *hsd, HAL_SD_CardCIDTypeDef *pCID);
HAL_StatusTypeDef       HAL_SD_GetCardCSD(SD_HandleTypeDef *hsd, HAL_SD_CardCSDTypeDef *pCSD);
HAL_StatusTypeDef       HAL_SD_GetCardStatus(SD_HandleTypeDef *hsd, HAL_SD_CardStatusTypeDef *pStatus);
HAL_StatusTypeDef       HAL_SD_GetCardInfo(SD_HandleTypeDef *hsd, HAL_SD_CardInfoTypeDef *pCardInfo);
/**
* @}
*/

/** @defgroup SD_Exported_Functions_Group5 Peripheral State and Errors functions
* @{
*/
HAL_SD_StateTypeDef HAL_SD_GetState(SD_HandleTypeDef *hsd);
uint32_t HAL_SD_GetError(SD_HandleTypeDef *hsd);
/**
* @}
*/

/** @defgroup SD_Exported_Functions_Group6 Perioheral Abort management
* @{
*/
HAL_StatusTypeDef HAL_SD_Abort(SD_HandleTypeDef *hsd);
HAL_StatusTypeDef HAL_SD_Abort_IT(SD_HandleTypeDef *hsd);
/**
* @}
*/

#endif

/**
  * @}
*/

/**
  * @}
  */

/**
  * @}
  */

#endif /* __BF0_HAL_SDHCI_EX_H__ */
