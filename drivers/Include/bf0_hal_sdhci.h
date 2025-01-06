/**
  ******************************************************************************
  * @file   bf0_hal_sdhci.h
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

#ifndef __BF0_HAL_SDHCI_H__
#define __BF0_HAL_SDHCI_H__

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"


/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup SDHCI SD Host Controller Interface
  * @{
  */

/* Register address and description define ------------------------------------------------------------*/
/** @defgroup SDHCI_Register_Types SDHCI Register Types
  * @{
  */

/*
 * Controller registers list, not use register table
 */

// TODO : move it to menuconfig later
//#define SDCARD_INSTANCE                   SDIO2

/******************** Register Description Start ************************************/
#ifndef __SDHCI_H

#define SDHCI_DMA_ADDRESS   0x00

#define SDHCI_BLOCK_SIZE    0x04
#define  SDHCI_MAKE_BLKSZ(dma, blksz) (((dma & 0x7) << 12) | (blksz & 0xFFF))

#define SDHCI_BLOCK_COUNT   0x06

#define SDHCI_ARGUMENT      0x08

#define SDHCI_TRANSFER_MODE 0x0C
#define  SDHCI_TRNS_DMA     0x01
#define  SDHCI_TRNS_BLK_CNT_EN  0x02
#define  SDHCI_TRNS_ACMD12  0x04
#define  SDHCI_TRNS_READ    0x10
#define  SDHCI_TRNS_MULTI   0x20

#define SDHCI_COMMAND       0x0E
#define  SDHCI_CMD_RESP_MASK    0x03
#define  SDHCI_CMD_CRC      0x08
#define  SDHCI_CMD_INDEX    0x10
#define  SDHCI_CMD_DATA     0x20

#define  SDHCI_CMD_RESP_NONE    0x00
#define  SDHCI_CMD_RESP_LONG    0x01
#define  SDHCI_CMD_RESP_SHORT   0x02
#define  SDHCI_CMD_RESP_SHORT_BUSY 0x03

#define SDHCI_MAKE_CMD(c, f) (((c & 0xff) << 8) | (f & 0xff))

#define SDHCI_RESPONSE      0x10

#define SDHCI_BUFFER        0x20

#define SDHCI_PRESENT_STATE 0x24
#define  SDHCI_CMD_INHIBIT  0x00000001
#define  SDHCI_DATA_INHIBIT 0x00000002
#define  SDHCI_DOING_WRITE  0x00000100
#define  SDHCI_DOING_READ   0x00000200
#define  SDHCI_SPACE_AVAILABLE  0x00000400
#define  SDHCI_DATA_AVAILABLE   0x00000800
#define  SDHCI_CARD_PRESENT 0x00010000
#define  SDHCI_WRITE_PROTECT    0x00080000

#define SDHCI_HOST_CONTROL  0x28
#define  SDHCI_CTRL_LED     0x01
#define  SDHCI_CTRL_4BITBUS 0x02
#define  SDHCI_CTRL_HISPD   0x04
#define  SDHCI_CTRL_DMA_MASK    0x18
#define  SDHCI_CTRL_8BITBUS    0x20
#define   SDHCI_CTRL_SDMA   0x00
#define   SDHCI_CTRL_ADMA1  0x08
#define   SDHCI_CTRL_ADMA32 0x10
#define   SDHCI_CTRL_ADMA64 0x18

#define SDHCI_POWER_CONTROL 0x29
#define  SDHCI_POWER_ON     0x01
#define  SDHCI_POWER_180    0x0A
#define  SDHCI_POWER_300    0x0C
#define  SDHCI_POWER_330    0x0E

#define SDHCI_BLOCK_GAP_CONTROL 0x2A

#define SDHCI_WAKE_UP_CONTROL   0x2B

#define SDHCI_CLOCK_CONTROL 0x2C
#define  SDHCI_DIVIDER_SHIFT    8
#define  SDHCI_CLOCK_CARD_EN    0x0004
#define  SDHCI_CLOCK_INT_STABLE 0x0002
#define  SDHCI_CLOCK_INT_EN 0x0001

#define SDHCI_TIMEOUT_CONTROL   0x2E

#define SDHCI_SOFTWARE_RESET    0x2F
#define  SDHCI_RESET_ALL    0x01
#define  SDHCI_RESET_CMD    0x02
#define  SDHCI_RESET_DATA   0x04

#define SDHCI_INT_STATUS    0x30
#define SDHCI_INT_ENABLE    0x34
#define SDHCI_SIGNAL_ENABLE 0x38
#define  SDHCI_INT_RESPONSE 0x00000001
#define  SDHCI_INT_DATA_END 0x00000002
#define  SDHCI_INT_DMA_END  0x00000008
#define  SDHCI_INT_SPACE_AVAIL  0x00000010
#define  SDHCI_INT_DATA_AVAIL   0x00000020
#define  SDHCI_INT_CARD_INSERT  0x00000040
#define  SDHCI_INT_CARD_REMOVE  0x00000080
#define  SDHCI_INT_CARD_INT 0x00000100
#define  SDHCI_INT_ERROR    0x00008000
#define  SDHCI_INT_TIMEOUT  0x00010000
#define  SDHCI_INT_CRC      0x00020000
#define  SDHCI_INT_END_BIT  0x00040000
#define  SDHCI_INT_INDEX    0x00080000
#define  SDHCI_INT_DATA_TIMEOUT 0x00100000
#define  SDHCI_INT_DATA_CRC 0x00200000
#define  SDHCI_INT_DATA_END_BIT 0x00400000
#define  SDHCI_INT_BUS_POWER    0x00800000
#define  SDHCI_INT_ACMD12ERR    0x01000000
#define  SDHCI_INT_ADMA_ERROR   0x02000000
#define   SDHCI_TUNING_EVENT    0x00001000 //added SD3.0
#define   SDHCI_TUNING_ERROR    0x08000000
//#define   SDHCI_TUNING_EVENT    0x00000000 //added SD3.0
//#define   SDHCI_TUNING_ERROR    0x00000000

#define  SDHCI_INT_NORMAL_MASK  0x00007FFF
#define  SDHCI_INT_ERROR_MASK   0xFFFF8000

#define  SDHCI_INT_CMD_MASK (SDHCI_INT_RESPONSE | SDHCI_INT_TIMEOUT | \
        SDHCI_INT_CRC | SDHCI_INT_END_BIT | SDHCI_INT_INDEX)
#define  SDHCI_INT_DATA_MASK    (SDHCI_INT_DATA_END | SDHCI_INT_DMA_END | \
        SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL | \
        SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_DATA_CRC | \
        SDHCI_INT_DATA_END_BIT | SDHCI_INT_ADMA_ERROR)
#define SDHCI_INT_ALL_MASK  ((unsigned int)-1)

#define SDHCI_ACMD12_ERR    0x3C

#define SDHCI_HOST_CONTROL2     0x3E
/* 3E-3F reserved */

#define SDHCI_CAPABILITIES  0x40
#define  SDHCI_TIMEOUT_CLK_MASK 0x0000003F
#define  SDHCI_TIMEOUT_CLK_SHIFT 0
#define  SDHCI_TIMEOUT_CLK_UNIT 0x00000080
#define  SDHCI_CLOCK_BASE_MASK  0x00003F00
#define  SDHCI_CLOCK_BASE_3     0x0000FF00
#define  SDHCI_CLOCK_BASE_SHIFT 8
#define  SDHCI_MAX_BLOCK_MASK   0x00030000
#define  SDHCI_MAX_BLOCK_SHIFT  16
#define  SDHCI_CAN_DO_ADMA2 0x00080000
#define  SDHCI_CAN_DO_ADMA1 0x00100000
#define  SDHCI_CAN_DO_HISPD 0x00200000
#define  SDHCI_CAN_DO_SDMA  0x00400000
#define  SDHCI_CAN_VDD_330  0x01000000
#define  SDHCI_CAN_VDD_300  0x02000000
#define  SDHCI_CAN_VDD_180  0x04000000
#define  SDHCI_CAN_64BIT    0x10000000

//eMMMC fields

#define eMMC_BOOT_ACK_INT      0x00002000
#define eMMC_BOOT_DONE_INT     0x00004000
//#define eMMC_BOOT_ACK_INT      0x00002000

/* 44-47 reserved for more caps */

#define SDHCI_MAX_CURRENT   0x48

/* 4C-4F reserved for more max current */

#define SDHCI_SET_ACMD12_ERROR  0x50
#define SDHCI_SET_INT_ERROR 0x52

#define SDHCI_ADMA_ERROR    0x54

/* 55-57 reserved */

#define SDHCI_ADMA_ADDRESS  0x58
#define SDHCI_DMA_NEXTADDR    0x5C

/* 60-FB reserved */

#define SDHCI_CLK_TUNE    0xF4
#define SDHCI_SLOT_INT_STATUS   0xFC

#define SDHCI_HOST_VERSION  0xFE
#define  SDHCI_VENDOR_VER_MASK  0xFF00
#define  SDHCI_VENDOR_VER_SHIFT 8
#define  SDHCI_SPEC_VER_MASK    0x00FF
#define  SDHCI_SPEC_VER_SHIFT   0
#define   SDHCI_SPEC_100    0
#define   SDHCI_SPEC_200    1
#define   SDHCI_SPEC_300    2

#endif
/******************** Register Description End ************************************/
/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup SDHCI_Exported_Types SDHCI Exported Types
  * @{
  */

#define SDHCI_ADMA_MAX_SIZE         (65536)        /*!< SDADC Max support transfer size  */

#define SDHCI_USE_SDMA      (1<<0)      /*!<  Host is SDMA capable */
#define SDHCI_USE_ADMA      (1<<1)      /*!<  Host is ADMA capable */
#define SDHCI_REQ_USE_DMA   (1<<2)      /*!<  Use DMA for this req. */
#define SDHCI_DEVICE_DEAD   (1<<3)      /*!<  Device unresponsive */

/**
  * @brief  ENUM definition of DMA transfer mode
  */
enum HAL_SD_DMA_MODE_E
{
    SDHCI_DMA_PIO    = 0,   /*!<  Host use cpu copy */
    SDHCI_SDMA     = 1,     /*!<  Host use single DMA */
    SDHCI_ADMA   = 2,       /*!<  Host use advance DMA */
    SDHCI_DMA_CNT,
};


/**
  * @brief  ENUM definition of aDMA attributes
  */
enum HAL_ATTR_ACT_E
{
    ATTR_ACT_NOP    = 0,    /*!<  nop */
    ATTR_ACT_RSV     = 1,       /*!<  reserved */
    ATTR_ACT_TRAN   = 2,        /*!<  normal transfer */
    ATTR_ACT_LINK   = 3,        /*!<  use link list */
};

/**
  * @brief  structure to fill adma descriptor
  */
typedef struct
{
    uint8_t attr;   /*!<  bit5 act2, 4 act1; bit 2 Int, 1 End, 0 Valid
                    ACT as enum ATTR_ACT_E, set as ATTR_ACT_E<<4
                    Int = 1 generate DMA interrupt when operation of des line is completed
                    End = 1 indicate end of descriptor, transfer complete interrupt generate ...
                    Valid = 1 indicate this line is effective, valid=0 will generate adma error interrupt */
    uint8_t reserved;   /*!<  reserved */
    uint16_t len;       /*!<  tranfer data length */
    uint32_t addr;      /*!<  SD address */
} hal_sdhci_adma_des_line_t;

/**
  * @brief  SDHCI Initial structure
  */
typedef struct
{
    uint32_t        flags;      /*!<   Host attributes */
    uint8_t         pwr;        /*!<   Current voltage */
    uint32_t        blocks;     /*!<   remaining PIO blocks */
    uint8_t         *adma_desc; /*!<   ADMA descriptor table */
} SDHCI_InitTypeDef;


/**
  * @brief  SDHCI command parameter
  */
typedef struct
{
    uint32_t        Arg;             /*!<   argument */
    uint32_t        CmdIndex;        /*!<   SD command  */
    uint32_t        RespType;        /*!<   Response */

} SDHCI_CmdArgTypeDef;

/**
  * @brief  SDHCI handle type define
  */
typedef struct
{
    uint32_t                        Instance;    /*!<   Mapped address */
    SDHCI_InitTypeDef               Init;        /*!<   ininial parameter */
    HAL_LockTypeDef                 Lock;        /*!<   hal lock */
    __IO uint32_t                   State;       /*!<   hal status */
    __IO uint32_t                   ErrorCode;       /*!<   error code */
} SDHCI_HandleTypeDef;

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @addtogroup SDHCI_Exported_Functions
  * @{
  */

/** @addtogroup SDHCI_Exported_Functions_Group1
  * @{
  */

/**
  * @brief Get SD host version.
  * @param handle SDHCI handle.
  * @retval host version
  */
uint16_t hal_sdhci_get_host_version(SDHCI_HandleTypeDef *handle);

/**
  * @brief Get SD host capbility.
  * @param handle SDHCI handle.
  * @retval host capbility
  */
uint32_t hal_sdhci_get_host_cap(SDHCI_HandleTypeDef *handle);

/**
  * @brief set irq mask  .
  * @param handle SDHCI handle.
  * @param set irq need to be set or clear.
  * @retval none
  */
void hal_sdhci_clear_mask_irqs(SDHCI_HandleTypeDef *handle, uint32_t set);

/**
  * @brief unmask irq .
  * @param handle SDHCI handle.
  * @param irqs irq bits.
  * @retval none
  */
void hal_sdhci_unmask_irqs(SDHCI_HandleTypeDef *handle, uint32_t irqs);

/**
  * @brief mask irq .
  * @param handle SDHCI handle.
  * @param irqs irq bits.
  * @retval none
  */
void hal_sdhci_mask_irqs(SDHCI_HandleTypeDef *handle, uint32_t irqs);

/**
  * @brief get irq value.
  * @param handle SDHCI handle.
  * @retval irq set bits
  */
uint32_t hal_sdhci_get_int_value(SDHCI_HandleTypeDef *handle);

/**
  * @brief Clear irq .
  * @param handle SDHCI handle.
  * @param irq irq need to be cleared.
  * @retval none
  */
void hal_sdhci_clear_int(SDHCI_HandleTypeDef *handle, uint32_t irq);

/**
  * @brief enable sd card detect.
  * @param handle SDHCI handle.
  * @retval none
  */
void hal_sdhci_enable_card_detection(SDHCI_HandleTypeDef *handle);

/**
  * @brief disable sd card detect.
  * @param handle SDHCI handle.
  * @retval none
  */
void hal_sdhci_disable_card_detection(SDHCI_HandleTypeDef *handle);

/**
  * @brief set time out value.
  * @param handle SDHCI handle.
  * @param timeo time out value.
  * @retval none
  */
void hal_sdhci_set_timeout(SDHCI_HandleTypeDef *handle, uint8_t timeo);

/**
  * @brief set ADMA address.
  * @param handle SDHCI handle.
  * @param addr address set to register.
  * @retval none
  */
void hal_sdhic_set_adma_addr(SDHCI_HandleTypeDef *handle, uint32_t addr);

/**
  * @brief set DMA address.
  * @param handle SDHCI handle.
  * @param addr address set to register.
  * @retval none
  */
void hal_sdhci_set_dma_addr(SDHCI_HandleTypeDef *handle, uint32_t addr);

/**
  * @brief reset ADMA address.
  * @param handle SDHCI handle.
  * @retval none
  */
void hal_sdhci_reset_dma_addr(SDHCI_HandleTypeDef *handle);

/**
  * @brief set SDHCI DMA MODE
  * @param handle SDHCI handle.
  * @param flag for DMA mode, include ADMA/SDMA/PIO.
  * @retval none
  */
void hal_sdhci_set_dma_mode(SDHCI_HandleTypeDef *handle, uint8_t flag);

/**
  * @brief set SDHCI transfer mode.
  * @param handle SDHCI handle.
  * @param mode DMA or not DMA.
  * @retval none
  */
void hal_sdhci_set_transfer_mode(SDHCI_HandleTypeDef *handle, uint16_t mode);

/**
  * @brief set boot mode.
  * @param handle SDHCI handle.
  * @param boot boot mode or not.
  * @retval none
  */
void hal_sdhci_set_boot_mode(SDHCI_HandleTypeDef *handle, uint16_t boot);

/**
  * @brief transmit block setting.
  * @param handle SDHCI handle.
  * @param blk_size each block size .
  * @param blk_cnt block number .
  * @retval none
  */
void hal_sdhci_set_blk(SDHCI_HandleTypeDef *handle, uint32_t blk_size, uint32_t blk_cnt);

/**
  * @brief get present state.
  * @param handle SDHCI handle.
  * @retval present state
  */
uint32_t hal_sdhci_get_present_state(SDHCI_HandleTypeDef *handle);

/**
  * @brief set sd bit width.
  * @param handle SDHCI handle.
  * @param width 1 bit or 4 bits.
  * @retval none
  */
void hal_sdhci_set_bus_width(SDHCI_HandleTypeDef *handle, uint8_t width);

/**
  * @brief set sd command.
  * @param handle SDHCI handle.
  * @param cmd SD command structure.
  * @retval none
  */
void hal_sdhci_send_command(SDHCI_HandleTypeDef *handle, SDHCI_CmdArgTypeDef *cmd);

/**
  * @brief get sd command response.
  * @param handle SDHCI handle.
  * @param res_id response index, 0 ~ 3.
  * @retval response data
  */
uint32_t hal_sdhci_get_response(SDHCI_HandleTypeDef *handle, int res_id);

/**
  * @brief read/write data without DMA.
  * @param handle SDHCI handle.
  * @param is_read read or write.
  * @param data data buffer.
  * @param size data size need to be transmit.
  * @retval none
  */
void hal_sdhci_transfer_pio(SDHCI_HandleTypeDef *handle, uint8_t is_read, uint8_t *data, uint32_t size);

/**
  * @brief prepare adma table.
  * @param handle SDHCI handle.
  * @param data data buffer.
  * @param size data size need to be transmit.
  * @retval none
  */
int hal_sdhci_adma_table_pre(SDHCI_HandleTypeDef *handle, uint8_t *data, uint32_t size);

/**
  * @brief enable irq of transfer bits.
  * @param handle SDHCI handle.
  * @retval none
  */
void hal_sdhci_set_transfer_irqs(SDHCI_HandleTypeDef *handle);

/**
  * @brief set SD power mode.
  * @param handle SDHCI handle.
  * @param power power mode.
  * @retval none
  */
void hal_sdhci_set_power(SDHCI_HandleTypeDef *handle, unsigned short power);

/**
  * @brief set SD clock .
  * @param handle SDHCI handle.
  * @param clock clock freq for SD.
  * @param max_clk max system clock input.
  * @retval none
  */
void hal_sdhci_set_clk(SDHCI_HandleTypeDef *handle, uint32_t clock, uint32_t max_clk);

/**
  * @brief set DDR mode for MMC .
  * @param handle SDHCI handle.
  * @param ddr DDR mode enable.
  * @retval none
  */
void hal_sdhci_set_ddr(SDHCI_HandleTypeDef *handle, uint8_t ddr);

/**
  * @}
  */

/** @addtogroup SDHCI_Exported_Functions_Group2
  * @{
  */

/**
  * @brief reset SDHCI .
  * @param handle SDHCI handle.
  * @param mask mask bit.
  * @retval none
  */
void hal_sdhci_reset(SDHCI_HandleTypeDef *handle, uint8_t mask);

/**
  * @brief initial sd hardware .
  * @param handle SDHCI handle.
  * @param soft just reset command and data.
  * @retval none
  */
void hal_sdhci_init(SDHCI_HandleTypeDef *handle, int soft);

/**
  * @brief reinitial sd with command and data .
  * @param handle SDHCI handle.
  * @retval none
  */
void hal_sdhci_reinit(SDHCI_HandleTypeDef *handle);

/**
  * @brief Wait command finish or error .
  * @param handle SDHCI handle.
  * @param timeout wait time out in ms.
  * @retval irq value, -1 if fail
  */
int hal_sdhci_command_finish(SDHCI_HandleTypeDef *handle, uint32_t timeout);

/**
  * @brief Wait data transfer finish or error .
  * @param handle SDHCI handle.
  * @param timeout wait time out in ms.
  * @retval irq value, -1 if fail
  */
int hal_sdhci_data_finish(SDHCI_HandleTypeDef *handle, uint32_t timeout);


/**
  * @brief Read SD data in fifo.
  * @param handle SDHCI handle.
  * @retval data in fifo
  */
uint32_t hal_sdhci_read_fifo(SDHCI_HandleTypeDef *handle);

/**
  * @brief Write data to SD fifo.
  * @param handle SDHCI handle.
  * @param value data with 32 bits.
  * @retval none
  */
void hal_sdhci_write_fifo(SDHCI_HandleTypeDef *handle, uint32_t value);

/**
  * @brief Initial SD msp.
  * @param handle SDHCI handle.
  * @retval none.
  */
void HAL_SDHCI_MspInit(SDHCI_HandleTypeDef *handle);

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

#endif /* __BF0_HAL_SDHCI_H__ */
