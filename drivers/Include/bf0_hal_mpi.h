/**
  ******************************************************************************
  * @file   bf0_hal_mpi.h
  * @author Sifli software development team
  * @brief   Header file of QSPI HAL module.
  *
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

#ifndef BF0_HAL_MPI_H
#define BF0_HAL_MPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

#include "mpi.h"

//#define FLASHC_TypeDef QSPI_TypeDef


/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup SPI_FLASH
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup SPI_FLASH_Exported_Types SPI_FLASH Exported Types
  * @{
  */

#define SPI_FLASH_OTP_BASE          (0X1000U)

/**
  * @brief  SPI_FLASH configure tyep
  */
typedef struct
{
    MPI_TypeDef        *Instance;    /*!< qspi controller instance        */
    uint32_t            line;       /*!<  0 normal spi, 1 dual spi, 2 qual spi */
    uint32_t            base;        /*!< flash base memory address */
    uint32_t            msize;      /*!<  size on MB */
    uint32_t            SpiMode;    /*!<  0 nor, 1 nand, 2 psram */
} qspi_configure_t;

/**
  * @brief  SPI_FLASH manual command
  */
typedef struct
{
    uint8_t cmd;
    uint8_t func_mode;   /*!<   manual read 0 / write 1  */
    uint8_t data_mode;   /*!<   0 no data / 1 single line / 2 dual line / 3 qual line  */
    uint8_t dummy_cycle;     /*!<   dummy cycle between command and address/data  */
    uint8_t ab_size;          /*!<   alternate byte size, */
    uint8_t ab_mode;          /*!<   alternate byte mode, 0 no, 1, single line, 2, dual line, 3 fouline */
    uint8_t addr_size;   /*!<   address byte count - 1 */
    uint8_t addr_mode;   /*!<   0 no, 1 single line / 2 dual lin / 3 four line */
    uint8_t ins_mode;     /*!<  Instruction mode: 0 - No instruction; 1 - single line; 2 - two lines; 3 - four lines; 4/5/6 - reserved; 7 - four lines DDR */
} FLASH_CMD_CFG_T;

/**
  * @brief  SPI_FLASH command index
  */
typedef enum
{
    SPI_FLASH_CMD_WREN = 0,  /*!<  write enable, nor+nand    */
    SPI_FLASH_CMD_WRDI,     /*!<  write disable, nor+nand    */
    SPI_FLASH_CMD_RDSR,     /*!<  read status register, nor+nand    */
    SPI_FLASH_CMD_WRSR,     /*!<  write status register, nor+nand    */
    SPI_FLASH_CMD_PREAD,    /*!<  page read, nand    */
    SPI_FLASH_CMD_READ,     /*!<  single line read, nor+nand    */
    SPI_FLASH_CMD_FREAD,    /*!<  fast read , nor + nand    */
    SPI_FLASH_CMD_DREAD,    /*!<  fast read dual output, nor+nand    */
    SPI_FLASH_CMD_QREAD,    /*!<  fast read quad output, nor+nand    */
    SPI_FLASH_CMD_2READ,    /*!<  2 line read, nor+nand    */
    SPI_FLASH_CMD_4READ,   /*!<  4 line read, nor+nand .   ==== 10    */
    SPI_FLASH_CMD_RDID,    /*!<  read id, nor+nand    */
    SPI_FLASH_CMD_PLD,     /*!<  load program data, nand    */
    SPI_FLASH_CMD_QPLD,    /*!<  qual program load, nand    */
    SPI_FLASH_CMD_PLDR,    /*!<  randome program load, nand    */
    SPI_FLASH_CMD_QPLDR,   /*!<  qual random program load, nand    */
    SPI_FLASH_CMD_PEXE,    /*!<  program execute, nand    */
    SPI_FLASH_CMD_BE,      /*!<  block erase, nand    */
    SPI_FLASH_CMD_RST,     /*!<  reset, nor+nand    */
    SPI_FLASH_CMD_RST_EN,     /*!<  reset en, nor    */
    SPI_FLASH_CMD_RDSR2,   /*!<  read status register 2, nor       ==== 20    */
    SPI_FLASH_CMD_WVSR,    /*!<  write volatile status register, nor    */
    SPI_FLASH_CMD_PP,      /*!<  PAGE PROGRAM, nor    */
    SPI_FLASH_CMD_QPP,     /*!<  QUAL PAGE PROGRAM, nor    */
    SPI_FLASH_CMD_RDEAR,     /*!<  read extended address register, nor    */
    SPI_FLASH_CMD_WREAR,     /*!<  write extended address register, nor    */
    SPI_FLASH_CMD_PE,        /*!<  page erase, nor    */
    SPI_FLASH_CMD_SE,        /*!<  SECTOR erase, nor    */
    SPI_FLASH_CMD_BE32,        /*!<  BLOCK erase 32KB, nor    */
    SPI_FLASH_CMD_BE64,        /*!<  BLOCK erase 64KB, nor    */
    SPI_FLASH_CMD_CE,        /*!<  CHIP ERASE, nor             ===== 30    */
    SPI_FLASH_CMD_RDSR3,   /*!<  read status register 3, nor    */
    SPI_FLASH_CMD_WRSR3,   /*!<  WRITE status register 3, nor    */
    SPI_FLASH_CMD_EN4BM,   /*!<  enter 4-byte address mode, nor    */
    SPI_FLASH_CMD_ET4BM,   /*!<  exit 4-byte address mode, nor    */
    SPI_FLASH_CMD_RD4BA,   /*!<  read with 4-byte address, nor    */
    SPI_FLASH_CMD_FR4BA, /*!<  fast read with 4-byte address, nor    */
    SPI_FLASH_CMD_FQR4BA, /*!<  fast read quad output with 4-byte address, nor    */
    SPI_FLASH_CMD_4RD4BA, /*!<  4 IO read with 4-byte address, nor    */
    SPI_FLASH_CMD_PP4BA, /*!<  page program with 4-byte address, nor    */
    SPI_FLASH_CMD_QPP4BA, /*!<  quad page program with 4-byte address, nor     ==== 40    */
    SPI_FLASH_CMD_SE4BA, /*!<  sector erase with 4-byte address, nor    */
    SPI_FLASH_CMD_BE4BA, /*!<  64KB block erase with 4-byte address, nor    */
    SPI_FLASH_CMD_WRSR2, /*!<  write status register command 2, nor    */
    SPI_FLASH_CMD_LEFPA, /*!< Last ECC Failue Page Address, NAND    */
    SPI_FLASH_CMD_BBM, /*!< Bad Block Management, NAND    */
    SPI_FLASH_CMD_RBLUT, /*!< Read BBM Look Up Table, NAND    */
    SPI_FLASH_CMD_CFREAD, /*!< Continue fast read with 1 line, NAND    */
    SPI_FLASH_CMD_C4READ, /*!< Continue read with 4 line, NAND    */
    SPI_FLASH_CMD_RUID, /*!< Read Unique ID, NOR    */
    SPI_FLASH_CMD_RDSCUR, /*!< Read Security Register, NOR                    ====== 50   */
    SPI_FLASH_CMD_PRSCUR, /*!< Program Security Register, NOR    */
    SPI_FLASH_CMD_ERSCUR, /*!< Erase Security Registers, NOR    */
    SPI_FLASH_CMD_DPD, /*!< Deep Power-Down, NOR    */
    SPI_FLASH_CMD_RDP, /*!< Release from Deep Power-Down, NOR    */
    SPI_FLASH_CMD_DTR4R, /*!< DTR 4 IO Read, NOR    */
    SPI_FLASH_CMD_RDSFDP, /*!< Read SFDP(Serial Flash Discoverable Parameter), NOR    */
    SPI_FLASH_CMD_COUNT /*!< current support flash command     */
} SPI_FLASH_CMD_E;

/**
  * @brief  SPI_FLASH command table
  */
typedef struct
{
    uint8_t flash_mode; /*!<  SPI_FLASH_MODE_E    */
    uint8_t manuf_id; /*!< FLASH_MANUF_ID_T , manufacturer ID   */
    uint8_t mem_type;   /*!< Memory type for NOR, device ID1 for NAND */
    uint8_t dev_id;     /*!<  Memory density for NOR, device ID2 for NAND    */
    uint8_t status_reg; /*!< status regiser address for nand  */
    uint8_t protect_reg;  /*!<  protect register address for nand*/
    uint8_t mode_reg;   /*!< qe/continue/ecc-e(ecc always bit 4)  mode register for nand,
                                otp page counter for nor */
    uint8_t ecc_sta_mask;   /*!< ecc status bits on status_reg for nand, protect bits on reg0 for nor */
    uint8_t qe_mod_mask;   /*!< qe setting bits on mode_reg for nand,
                                    qe register and bit for nor(bit_cnt<<4 | reg_cnt) */
    uint8_t buf_mod_mask;   /*!< buf/continue mode bits on mode_reg */
    uint8_t ecc_en_mask;        /*!< ecc enable bits on mode_reg */
    uint8_t oob_size;  /*!<  oob size  for nand, otp page size for each otp (size_in_byte>>8)*/
    uint32_t erase_base_size; /*!< min   erase size */
    FLASH_CMD_CFG_T cmd_cfg[SPI_FLASH_CMD_COUNT];  /*!<  command table */
} SPI_FLASH_FACT_CFG_T;

/**
  * @brief  SPI_FLASH manufactory id
  */
typedef enum
{
    FLASH_INVALID_ID = 0x00,    /*!< Initial id        */
    FLASH_SPANSION = 0x01,    /*!< Spansion     */
    FLASH_PUYA = 0X85,      /*!< puya       */
    FLASH_EON = 0X1C,       /*!< EON    */
    FLASH_MXIC = 0XC2,      /*!< Mxic    */
    FLASH_GIGA = 0XC8,       /*!< Giga device     */
    FLASH_WINBOND = 0XEF,       /*!< winbond    */
    FLASH_FIDELIX = 0XF8,   /*!< Fidelix     */
    FLASH_ESMT = 0X8C,        /*!< Esmt  */
    FLASH_NUMONYX = 0X20,     /*!< numonyx       */
    FLASH_MICRON = 0X2C,    /*!< micro      */
    FLASH_PM25L = 0X7F,     /*!< pm25l     */
    FLASH_FUDAN = 0XA1,     /*!< fudan      */
    FLASH_HYF = 0XC9,     /*!< fudan      */
    FLASH_UNKNOW_ID = 0XFF, /*!< Invalid        */
} FLASH_MANUF_ID_T;

/**
  * @brief  SPI_FLASH status define
  */
typedef enum
{
    HAL_FLASH_STATE_RESET      = 0x00U,    /*!< Peripheral not Initialized                         */
    HAL_FLASH_STATE_READY      = 0x01U,    /*!< Peripheral Initialized and ready for use           */
    HAL_FLASH_STATE_BUSY       = 0x02U,    /*!< an internal process is ongoing                     */
    HAL_FLASH_STATE_ERROR      = 0x03U,    /*!< FLASH error state                                    */
    HAL_FLASH_STATE_ABORT      = 0x04U     /*!< FLASH abort is ongoing                               */
} HAL_FLASH_StateTypeDef;

/**
  * @brief  SPI_FLASH spi line mode
  */
typedef enum
{
    HAL_FLASH_NOR_MODE      = 0x00U,     /*!< Flash single spi mode      */
    HAL_FLASH_DMODE      = 0x01U,        /*!< Flash dual spi mode      */
    HAL_FLASH_QMODE       = 0x02U,       /*!< Flash qual spi mode      */
} HAL_FLASH_ModeDef;

/**
  * @brief  SPI_FLASH AES mode
  */
typedef enum
{
    HAL_FLASH_AES128      = 0x00U,  /*!<  AES 128      */
    HAL_FLASH_AES256      = 0x01U,  /*!<  AES 256      */
} HAL_FLASH_AES_Mode;

/**
  * @brief  SPI_FLASH flash type
  */
typedef enum
{
    SPI_MODE_NOR = 0,         /*!< Flash TYPE NOR mode      */
    SPI_MODE_NAND = 1,          /*!< Flash TYPE NAND mode      */
    SPI_MODE_PSRAM = 2,     /*!< Flash TYPE QSPI PSRAM mode      */
    SPI_MODE_OPSRAM = 3,     /*!< Flash TYPE OPI PSRAM mode      */
    SPI_MODE_HPSRAM = 4,     /*!< Flash TYPE HPI PSRAM mode      */
    SPI_MODE_LEGPSRAM = 5,     /*!< Flash TYPE LEGACY PSRAM mode    */
    SPI_MODE_HBPSRAM = 6,     /*!< Flash TYPE Hyper Bus PSRAM mode    */
    SPI_MODE_COUNT
} SPI_FLASH_MODE_E;

/**
  * @brief  SPI_FLASH clear fifo mode
  */
typedef enum
{
    HAL_FLASH_CLR_RX_FIFO = 1,      /*!< Clear flash RX FIFO      */
    HAL_FLASH_CLR_TX_FIFO = 2,         /*!< Clear Flash TX FIFO      */
    HAL_FLASH_CLR_RX_TX_FIFO = 3,   /*!< Clear Flash RX AND TX FIFO     */
} HAL_FLASH_ClearFifo;

/**
  * @brief  SPI_FLASH CS control function type
  */
typedef void (*flash_cs_ctrl)(uint32_t pulldown);
typedef void (*flash_lock_ctrl)(uint32_t addr, uint32_t lock);

#define NAND_FLAG_PAGE_DOUBLE       1
#define NAND_FLAG_BLOCK_DOUBLE      2
/**
  * @brief  SPI_FLASH initial structure definition
  */
typedef struct __FLASH_HandleTypeDef
{
    MPI_TypeDef                      *Instance;     /*!< MPI hardware handler   */
    uint32_t                     ErrorCode;    /*!< MPI error code   */
    DMA_HandleTypeDef                 *dma;         /*!< DMA handler */
    const SPI_FLASH_FACT_CFG_T      *ctable;         /*!<flash command timeing list  */
    uint32_t                          base;         /*!< base address in memory map  */
    uint32_t                          size;         /*!< flash/psram size in bytes  */
    uint32_t                        freq;             /*!< Flash/Psram running frequency  */
    uint8_t                         *data_buf;      /*!< nand cache buffer  */
    HAL_FLASH_ModeDef                 Mode;         /*!< spi, dual spi or qual spi */
    HAL_LockTypeDef                   Lock;         /*!< Lock not used   */
    HAL_FLASH_StateTypeDef       State;         /*!< MPI status   */
    uint8_t                          isNand;        /*!< nor, nand or psram  */
    uint8_t                          dualFlash;     /*!< used for control flag:
                                                    * bit 0 for nand bytes per page: 0 for 2048 bytes, 1 for 4096 bytes.
                                                    * bit 1 for nand pages per block: 0 for 64 pages, 1 for 128 pages
                                                    * bit 4~7: for NAND ECC status mode as NAND_ECC_MODE_T */
    uint8_t                         ecc_en;         /*!< enable ecc for nand; configure rxclk inv/delay for nor; read latency for psram */
    uint8_t                         buf_mode;       /*!< enable buffer mode for nand; enable DTR for nor; write latency for psram  */
    uint8_t                         wakeup;          /*!< wake up mode for psram, plane select flag for nand */
    uint32_t                        reserv1;         /*!< used as local clock divider for dual flash  */
    flash_cs_ctrl                   cs_ctrl;         /*!< cs control function pointer  */
    flash_lock_ctrl                 lock;
} FLASH_HandleTypeDef;
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/

/** @defgroup SPI_FLASH_Exported_Macros SPI_FLASH Exported Macros
  * @{
  */

/**
  * @brief QSPI Enable Hardware Interface .
  * @param  \__HANDLE__ SPI FLASH handle
  * @retval None
  */
#define __HAL_QSPI_EN_HWI(__HANDLE__)        (((__HANDLE__)->Instance->CR) |= MPI_CR_HWIFE)

/**
  * @brief QSPI Disable Hardware Interface .
  * @param  \__HANDLE__ SPI FLASH handle
  * @retval None
  */
#define __HAL_QSPI_DIS_HWI(__HANDLE__)        (((__HANDLE__)->Instance->CR) &= ~(MPI_CR_HWIFE))

/**
  * @brief QSPI Set Command , do not wait TCF.
  * @param  \__HANDLE__ SPI FLASH handle
  * @param  \__CMD__ Command for FLASH or PSRAM
  * @retval None
  */
#define __HAL_QSPI_SET_CMD(__HANDLE__, __CMD__)        (((__HANDLE__)->Instance->CMDR1) = (__CMD__&0XFF))


/**
  * @brief Get MPI CR enable status.
  * @param  \__HANDLE__ MPI handle
  * @retval status
  */
#define __HAL_MPI_GET_CREN(__HANDLE__)        (((__HANDLE__)->Instance->CR) & (MPI_CR_EN_Msk))

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SPI_FLASH_Exported_Functions
  * @{
  */

/** @addtogroup SPI_FLASH_Exported_Functions_Group1
  * @{
  */

/**
  * @brief  SPI_FLASH set ahb read command
  * @param  hflash  SPI_FLASH handle
  * @param  cmd read command index
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FLASH_SET_AHB_RCMD(FLASH_HandleTypeDef *hflash, uint8_t cmd);

/**
  * @brief  SPI_FLASH set ahb write command
  * @param  hflash  SPI_FLASH handle
  * @param  cmd write command index
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FLASH_SET_AHB_WCMD(FLASH_HandleTypeDef *hflash, uint8_t cmd);

/**
  * @brief  SPI_FLASH configure AHB read command
  * @param  hflash  SPI_FLASH handle
  * @param  data_mod  data mode, 1 for single line, 2 for dual line, 3 for 4line
  * @param  dum_cyc  dummy cycle
  * @param  ab_size  ab size in bit
  * @param  ab_mod  ab mode, 1/2/4 line
  * @param  addr_size  address size, 0 for 1 byte ... 3 for 4 bytes
  * @param  addr_mod  address mode, 1/2/4 line, 1 for single line, 2 for dual line, 3 for 4line
  * @param  cmd_mode  command mode, it should be 1
  * @retval None
  */
void HAL_FLASH_CFG_AHB_RCMD(FLASH_HandleTypeDef *hflash, int8_t data_mod,
                            int8_t dum_cyc, int8_t ab_size, int8_t ab_mod,
                            int8_t addr_size, int8_t addr_mod, int8_t cmd_mode);

/**
  * @brief  SPI_FLASH config AHB write command
  * @param  hflash  SPI_FLASH handle
  * @param  data_mod  data mode, 1 for single line, 2 for dual line, 3 for 4line
  * @param  dum_cyc  dummy cycle
  * @param  ab_size  ab size in bit
  * @param  ab_mod  ab mode, 1/2/4 line
  * @param  addr_size  address size, 0 for 1 byte ... 3 for 4 bytes
  * @param  addr_mod  address mode, 1/2/4 line, 1 for single line, 2 for dual line, 3 for 4line
  * @param  cmd_mode  command mode, it should be 1
  * @retval None
  */
void HAL_FLASH_CFG_AHB_WCMD(FLASH_HandleTypeDef *hflash, int8_t data_mod,
                            int8_t dum_cyc, int8_t ab_size, int8_t ab_mod,
                            int8_t addr_size, int8_t addr_mod, int8_t cmd_mode);

/**
  * @brief  SPI_FLASH enable QSPI mode
  * @param  hflash  SPI_FLASH handle
  * @param  en enable QSPI mode
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FLASH_ENABLE_QSPI(FLASH_HandleTypeDef *hflash, uint8_t en);

/**
  * @brief  SPI_FLASH enable OPI mode
  * @param  hflash  SPI_FLASH handle
  * @param  en enable OPI mode
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FLASH_ENABLE_OPI(FLASH_HandleTypeDef *hflash, uint8_t en);

/**
 * @brief  SPI_FLASH for HyperBus psram
 * @param  hflash  FLASH handle
 * @param  en enable hyper
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_ENABLE_HYPER(FLASH_HandleTypeDef *hflash, uint8_t en);

/**
  * @brief  SPI_FLASH set flash command to register
  * @param  hflash  SPI_FLASH handle
  * @param  cmd command index
  * @param  addr address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FLASH_SET_CMD(FLASH_HandleTypeDef *hflash, uint8_t cmd, uint32_t addr);

/**
  * @brief  Configure FLASH command but do not need wait TCF.
  * @param  hflash  SPI_FLASH handle
  * @param  cmd command index
  * @param  addr address
  * @param  is_cmd2 set command 2 or command 1
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FLASH_CFG_CMD(FLASH_HandleTypeDef *hflash, uint8_t cmd, uint32_t addr, uint8_t is_cmd2);

/**
  * @brief  SPI_FLASH write data length
  * @param  hflash  FLASH handle
  * @param  len data length
  * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_WRITE_DLEN(FLASH_HandleTypeDef *hflash, uint32_t len);

/**
  * @brief  SPI_FLASH write data length to command 2
  * @param  hflash  FLASH handle
  * @param  len data length
  * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_WRITE_DLEN2(FLASH_HandleTypeDef *hflash, uint16_t len);

/**
  * @brief  SPI_FLASH write a word to fifo
  * @param  hflash  FLASH handle
  * @param  data u32 data
  * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_WRITE_WORD(FLASH_HandleTypeDef *hflash, uint32_t data);

/**
  * @brief  SPI_FLASH write alternate byte
  * @param  hflash  FLASH handle
  * @param  ab u32 data
  * @retval none
*/
void HAL_FLASH_WRITE_ABYTE(FLASH_HandleTypeDef *hflash, uint32_t ab);

/**
  * @brief  SPI_FLASH is rx fifo empty
  * @param  hflash  FLASH handle
  * @retval fifo status
*/
bool HAL_FLASH_IS_RX_EMPTY(FLASH_HandleTypeDef *hflash);

/**
  * @brief  SPI_FLASH is tx fifo full
  * @param  hflash  FLASH handle
  * @retval fifo status
*/
bool HAL_FLASH_IS_TX_FULL(FLASH_HandleTypeDef *hflash);

/**
  * @brief  SPI_FLASH is tx fifo empty
  * @param  hflash  FLASH handle
  * @retval fifo status
*/
bool HAL_FLASH_IS_TX_EMPTY(FLASH_HandleTypeDef *hflash);

/**
  * @brief  SPI_FLASH is busy
  * @param  hflash  FLASH handle
  * @retval flash status
*/
bool HAL_FLASH_IS_BUSY(FLASH_HandleTypeDef *hflash);

/**
  * @brief  SPI_FLASH status match previous setting
  * @param  hflash  FLASH handle
  * @retval flash status
*/
bool HAL_FLASH_STATUS_MATCH(FLASH_HandleTypeDef *hflash);

/**
  * @brief  SPI_FLASH if command finish
  * @param  hflash  FLASH handle
  * @retval flash status
*/

bool HAL_FLASH_IS_CMD_DONE(FLASH_HandleTypeDef *hflash);

/**
  * @brief  SPI_FLASH clear command done status
  * @param  hflash  FLASH handle
  * @retval none
*/
void HAL_FLASH_CLR_CMD_DONE(FLASH_HandleTypeDef *hflash);

/**
  * @brief  SPI_FLASH check if program done
  * @param  hflash  FLASH handle
  * @retval flash status
*/
bool HAL_FLASH_IS_PROG_DONE(FLASH_HandleTypeDef *hflash);

/**
  * @brief  SPI_FLASH read 32 bits data from fifo
  * @param  hflash  FLASH handle
  * @retval data in fifo
*/
uint32_t HAL_FLASH_READ32(FLASH_HandleTypeDef *hflash);

/**
  * @brief  SPI_FLASH get received count
  * @param  hflash  FLASH handle
  * @retval rx count
*/
uint8_t HAL_FLASH_GET_RX_CNT(FLASH_HandleTypeDef *hflash);

/**
  * @brief  SPI_FLASH set clock divider
  * @param  hflash  FLASH handle
  * @param  div clock divider
  * @retval none
*/
void HAL_FLASH_SET_CLK_rom(FLASH_HandleTypeDef *hflash, uint8_t div);

/**
  * @brief  SPI_FLASH flush flash fifo
  * @param  hflash  FLASH handle
  * @param  mode RX or tx OR ALL
  * @retval none
*/
void HAL_FLASH_CLEAR_FIFO(FLASH_HandleTypeDef *hflash, HAL_FLASH_ClearFifo mode);

/**
  * @brief  SPI_FLASH set tx slot to meet DMA request
  * @param  hflash  FLASH handle
  * @param  slot burst width
  * @retval none
*/
void HAL_FLASH_SET_TXSLOT(FLASH_HandleTypeDef *hflash, uint8_t slot);


/**
  * @brief  SPI_FLASH config command parameter
  * @param  hflash  SPI_FLASH handle
  * @param  func_mod function mode, read or write
  * @param  data_mod  data mode, 1 for single line, 2 for dual line, 3 for 4line
  * @param  dum_cyc  dummy cycle
  * @param  ab_size  ab size in bit
  * @param  ab_mod  ab mode, 1/2/4 line
  * @param  addr_size  address size, 0 for 1 byte ... 3 for 4 bytes
  * @param  addr_mod  address mode, 1/2/4 line, 1 for single line, 2 for dual line, 3 for 4line
  * @param  cmd_mode  command mode, it should be 1
  * @retval None
  */
void HAL_FLASH_MANUAL_CMD(FLASH_HandleTypeDef *hflash, int8_t func_mod, int8_t data_mod,
                          int8_t dum_cyc, int8_t ab_size, int8_t ab_mod,
                          int8_t addr_size, int8_t addr_mod, int8_t cmd_mode);


/**
    * @brief  SPI_FLASH config command parameter for command 2
    * @param  hflash  SPI_FLASH handle
    * @param  func_mod function mode, read or write
    * @param  data_mod  data mode, 1 for single line, 2 for dual line, 3 for 4line
    * @param  dum_cyc  dummy cycle
    * @param  ab_size  ab size in bit
    * @param  ab_mod  ab mode, 1/2/4 line
    * @param  addr_size  address size, 0 for 1 byte ... 3 for 4 bytes
    * @param  addr_mod  address mode, 1/2/4 line, 1 for single line, 2 for dual line, 3 for 4line
    * @param  cmd_mode  command mode, it should be 1
    * @retval None
  */
void HAL_FLASH_MANUAL_CMD2(FLASH_HandleTypeDef *hflash, int8_t func_mod, int8_t data_mod,
                           int8_t dum_cyc, int8_t ab_size, int8_t ab_mod,
                           int8_t addr_size, int8_t addr_mod, int8_t cmd_mode);

/**
 * @brief  SPI_FLASH enable/disable command 2
 * @param  hflash  FLASH handle
 * @param  en enable
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_ENABLE_CMD2(FLASH_HandleTypeDef *hflash, uint8_t en);

/**
 * @brief  SPI_FLASH command 2 match status en
 * @param  hflash  FLASH handle
 * @param  en enable
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_STAUS_MATCH_CMD2(FLASH_HandleTypeDef *hflash, uint8_t en);

/**
  * @brief  SPI_FLASH clear flash status
  * @param  hflash  FLASH handle
  * @param  status value
  * @retval none
*/
void HAL_FLASH_CLR_STATUS(FLASH_HandleTypeDef *hflash, uint32_t status);

/**
  * @}
  */


/** @addtogroup SPI_FLASH_Exported_Functions_Group2
  * @{
  */

/**
 * @brief  SPI_FLASH for boot mode
 * @param  hflash  FLASH handle
 * @param  start start address
 * @param  len data length
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_SET_ALIAS_RANGE(FLASH_HandleTypeDef *hflash, uint32_t start, uint32_t len);

/**
 * @brief  SPI_FLASH for boot mode
 * @param  hflash  FLASH handle
 * @param  offset alias offset
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_SET_ALIAS_OFFSET(FLASH_HandleTypeDef *hflash, uint32_t offset);

/**
 * @brief  SPI_FLASH for boot mode
 * @param  hflash  FLASH handle
 * @param  start start address
 * @param  end end address
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_SET_CTR(FLASH_HandleTypeDef *hflash, uint32_t start, uint32_t end);

/**
 * @brief  SPI_FLASH for boot mode
 * @param  hflash  FLASH handle
 * @param  data data buffer
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_SET_NONCE(FLASH_HandleTypeDef *hflash, uint8_t *data);

/**
 * @brief  SPI_FLASH for boot mode
 * @param  hflash  FLASH handle
 * @param  mode AES mode
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_SET_AES(FLASH_HandleTypeDef *hflash, HAL_FLASH_AES_Mode mode);

/**
 * @brief  SPI_FLASH for boot mode
 * @param  hflash  FLASH handle
 * @param  en enable
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_ENABLE_AES(FLASH_HandleTypeDef *hflash, uint8_t en);

/**
 * @brief  SPI_FLASH for boot mode
 * @param  hflash  FLASH handle
 * @param  cslmax cs low max
 * @param  cslmin cs low min
 * @param  cshmin cs high min
 * @param  trcmin Write/Read cycle minimum time
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_SET_CS_TIME(FLASH_HandleTypeDef *hflash,
                                        uint16_t cslmax, uint16_t cslmin, uint16_t cshmin, uint16_t trcmin);


/**
 * @brief  SPI_FLASH for boot mode
 * @param  hflash  FLASH handle
 * @param  rb_size row boundary
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_SET_ROW_BOUNDARY(FLASH_HandleTypeDef *hflash, uint8_t rb_size);

/**
 * @brief  SPI_FLASH for legacy psram
 * @param  hflash  FLASH handle
 * @param  en enable legacy
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_SET_LEGACY(FLASH_HandleTypeDef *hflash, uint8_t en);

/**
 * @brief  SPI_FLASH get clock divider
 * @param  hflash  FLASH handle
 * @retval clock divider
*/
uint8_t HAL_FLASH_GET_DIV(FLASH_HandleTypeDef *hflash);

/**
 * @brief  SPI_FLASH set dual flash mode
 * @param  hflash  FLASH handle
 * @param  dual 1 for dual flash, 0 for single
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_SET_DUAL_MODE(FLASH_HandleTypeDef *hflash, uint8_t dual);

/**
 * @brief  SPI_FLASH get dual flash mode
 * @param  hflash  FLASH handle
 * @retval 0 for single and 1 for dual flash mode
*/
int HAL_FLASH_GET_DUAL_MODE(FLASH_HandleTypeDef *hflash);

/**
 * @brief  Set MPI to X16 mode
 * @param  hflash  FLASH handle
 * @param  en enable X16 mode if 1
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_SET_X16_MODE(FLASH_HandleTypeDef *hflash, uint8_t en);

/**
 * @brief  MPI Indicate PSRAM is fixed latency or variable latency
 * @param  hflash  FLASH handle
 * @param  fix fixed latency or variable latency
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_MPI_EN_FIXLAT(FLASH_HandleTypeDef *hflash, uint8_t fix);

/**
 * @brief  MPI HyperBus protocol
 * @param  hflash  FLASH handle
 * @param  hyper hyperbus protocol
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_MPI_SET_HYPER(FLASH_HandleTypeDef *hflash, uint8_t hyper);

/**
 * @brief  MPI DQS enable.
 * @param  hflash  FLASH handle
 * @param  dqs Setting to 1 indicates device provides DQS signal for Rx data latching
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_MPI_ENABLE_DQS(FLASH_HandleTypeDef *hflash, uint8_t dqs);

/**
 * @brief  MPI select delayed version of DQS.
 * @param  hflash  FLASH handle
 * @param  delay delay version
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_MPI_SET_DQS_DELAY(FLASH_HandleTypeDef *hflash, uint8_t delay);

/**
 * @brief  MPI set SCK info
 * @param  hflash  FLASH handle
 * @param  delay Add delay on SCK (fine tune)
 * @param  inv Invert FCLK as SCK (coarse tune)
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_MPI_SET_SCK(FLASH_HandleTypeDef *hflash, uint8_t delay, uint8_t inv);

/**
 * @brief  MPI Enable pre-sampling for DTR
 * @param  hflash  FLASH handle
 * @param  en enable dtr
 * @param  rclk_inv_delay high 4 bits for rx clock invert, low 4 bits for rx clock delay
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_MPI_CFG_DTR(FLASH_HandleTypeDef *hflash, uint8_t en, uint8_t rclk_inv_delay);

/**
 * @brief  MPI modify AHB read delay cycle
 * @param  hflash  FLASH handle
 * @param  dum_cyc dummy cycle
 * @retval HAL status
*/
void HAL_MPI_MODIFY_RCMD_DELAY(FLASH_HandleTypeDef *hflash, int8_t dum_cyc);

/**
 * @brief  MPI modify AHB write delay cycle
 * @param  hflash  FLASH handle
 * @param  dum_cyc dummy cycle
 * @retval HAL status
*/
void HAL_MPI_MODIFY_WCMD_DELAY(FLASH_HandleTypeDef *hflash, int8_t dum_cyc);

/**
 * @brief  SPI_FLASH set watch dog timeout
 * @param  hflash  FLASH handle
 * @param  timeout timeout value in cycle
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_SET_WDT(FLASH_HandleTypeDef *hflash, uint16_t timeout);

/**
 * @brief  SPI_FLASH get wdt timeout configure value
 * @param  hflash  FLASH handle
 * @retval timeout value in cycle
*/
int HAL_FLASH_GET_WDT_VALUE(FLASH_HandleTypeDef *hflash);

/**
 * @brief  SPI_FLASH get wdt timeout flag
 * @param  hflash  FLASH handle
 * @retval 1 if timout
*/
int HAL_FLASH_GET_WDT_STATUS(FLASH_HandleTypeDef *hflash);

/**
 * @brief  SPI_FLASH config default ahb read command
 * @param  hflash  FLASH handle
 * @param  qmode qual spi mode
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_CONFIG_AHB_READ(FLASH_HandleTypeDef *hflash, bool qmode);

/**
 * @brief  SPI_FLASH set command parameter but not set command
 * @param  handle  FLASH handle
 * @param  cmd flash command
 * @retval 0 if success
*/
int HAL_FLASH_PRE_CMD(FLASH_HandleTypeDef *handle, SPI_FLASH_CMD_E cmd);

/**
 * @brief  SPI_FLASH issue flash command
 * @param  handle  FLASH handle
 * @param  cmd flash command
 * @param  addr flash address
 * @retval 0 if success
*/
int HAL_FLASH_ISSUE_CMD(FLASH_HandleTypeDef *handle, SPI_FLASH_CMD_E cmd, uint32_t addr);

/**
 * @brief  SPI_FLASH issue command 1 and command 2 in sequence
 * @param  handle  FLASH handle
 * @param  cmd1 flash command 1
 * @param  addr address for command 1
 * @param  cmd2 flash command 2
 * @param  param status mask value
 * @retval 0 if success
*/
int HAL_FLASH_ISSUE_CMD_SEQ(FLASH_HandleTypeDef *handle, SPI_FLASH_CMD_E cmd1, uint32_t addr, SPI_FLASH_CMD_E cmd2, uint32_t param);

/**
  * @}
  */

/** @addtogroup SPI_FLASH_Exported_Functions_Group3
  * @{
  */

/**
 * @brief  SPI_FLASH set qual spi mode
 * @param  hflash  FLASH handle
 * @param  en enable qual spi
 * @retval none
*/
void HAL_FLASH_SET_QUAL_SPI(FLASH_HandleTypeDef *hflash, bool en);

/**
 * @brief  SPI_FLASH set qual spi mode for 4 bytes address mode
 * @param  hflash  FLASH handle
 * @param  en enable qusl spi
 * @retval none
*/

void HAL_FLASH_FADDR_SET_QSPI(FLASH_HandleTypeDef *hflash, bool en);

/**
 * @brief  SPI_FLASH get nor flash device id
 * @param  hflash  FLASH handle
 * @retval nor flash device id
*/
int HAL_FLASH_GET_NOR_ID(FLASH_HandleTypeDef *hflash);

/**
 * @brief  SPI_FLASH check if duall flash mode by check flash id
 * @param  hflash  FLASH handle
 * @retval 0 if single and 1 if dual flash mode
*/
int HAL_FLASH_ID_DUAL_ID(FLASH_HandleTypeDef *hflash);

/**
 * @brief  SPI_FLASH volatile clear protect bits
 * @param  hflash  FLASH handle
 * @retval none
*/
void HAL_FLASH_CLR_PROTECT(FLASH_HandleTypeDef *hflash);

/**
 * @brief  SPI_FLASH initial controller
 * @param  hflash  FLASH handle
 * @param  cfg configurations
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_QSPI_Init(FLASH_HandleTypeDef *hflash, qspi_configure_t *cfg);

/**
 * @brief  SPI_FLASH pre initial, force set to NOR flash single mode.
           Only support AHB and OTP read after pre init.
 * @param  hflash  FLASH handle
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_PreInit(FLASH_HandleTypeDef *hflash);

/**
 * @brief  Get QSPI PSRAM chip id
 * @note   Data layout: densiy | KGD | MFID
 * @param  hqspi  qspi handle
 * @retval psram MFID & KGD & density
*/
int HAL_QSPI_GET_PSRAM_ID(FLASH_HandleTypeDef *hqspi);

/**
 * @brief  Set RX delay for high speed flash clock
 * @param[in]  qspi_id id for qspi interface, 0 for qspi1, 1 for qspi2, 2 for qspi3, 3 for qspi4, others reserved.
 * @param[in]  sen_delay select delayed version of Rx sample en
 * @param[in]  clk_delay Select delayed version of Rx clock in order to achieve higher operating frequency
 * @retval none
*/
void HAL_QSPI_SET_RXDELAY(uint8_t qspi_id, uint8_t sen_delay, uint8_t clk_delay);

/**
 * @brief  Set RX clk invert and delay for high speed flash clock
 * @param[in]  hflash  flash handle.
 * @param[in]  clk_inv select delayed invert
 * @param[in]  clk_delay Select delayed version of Rx clock in order to achieve higher operating frequency
 * @retval none
*/
void HAL_QSPI_SET_CLK_INV(FLASH_HandleTypeDef *hflash, uint8_t clk_inv, uint8_t clk_delay);

/**
 * @brief  FLASH into deep power down mode
 * @param  hflash  flash handle
 * @retval 0 if success
*/
int HAL_FLASH_DEEP_PWRDOWN(FLASH_HandleTypeDef *hflash);

/**
 * @brief  FLASH release from deep power down mode
 * @param  hflash  flash handle
 * @retval 0 if success
*/
int HAL_FLASH_RELEASE_DPD(FLASH_HandleTypeDef *hflash);


/**
 * @brief  Set comand1/command2 interval timer.
 * @param  hflash  FLASH handle
 * @param  val12 interval between command 1 and command 2
 * @param  val21 interval between command 2 and command 1
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_SET_INTERVAL(FLASH_HandleTypeDef *hflash, uint32_t val12, uint32_t val21);

/**
 * @brief  Enable FLASH hardware DMA interface.
 * @param  hflash  FLASH handle
 * @param  en 1 enalbe or 0 disable
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_DMA_EN(FLASH_HandleTypeDef *hflash, uint8_t en);

/**
 * @brief  Set comand1/command2 auto loop times.
 * @param  hflash  FLASH handle
 * @param  loop loop times
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_FLASH_SET_LOOP(FLASH_HandleTypeDef *hflash, uint32_t loop);


/**
 * @brief  Set mpi prefetch address.
 * @param  hflash  mpi handle
 * @param  start start address, need 1024 aligned
 * @param  len address length, need 1024 aligned
 * @retval HAL status
*/

HAL_StatusTypeDef HAL_MPI_CFG_PREFETCH(FLASH_HandleTypeDef *hflash, uint32_t start, uint32_t len);

/**
 * @brief  Enable/disable prefetch function.
 * @param  hflash  mpi handle
 * @param  en enable (1) or disable (0) prefetch
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_MPI_EN_PREFETCH(FLASH_HandleTypeDef *hflash, uint8_t en);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* BF0_HAL_QSPI_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
