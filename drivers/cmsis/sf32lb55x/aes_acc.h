/**
  ******************************************************************************
  * @file   aes_acc.h
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

#ifndef __AES_ACC_H
#define __AES_ACC_H

typedef struct
{
    __IO uint32_t COMMAND;
    __IO uint32_t STATUS;
    __IO uint32_t IRQ;
    __IO uint32_t SETTING;
    __IO uint32_t AES_SETTING;
    __IO uint32_t DMA_IN;
    __IO uint32_t DMA_OUT;
    __IO uint32_t DMA_DATA;
    __IO uint32_t IV_W0;
    __IO uint32_t IV_W1;
    __IO uint32_t IV_W2;
    __IO uint32_t IV_W3;
    __IO uint32_t EXT_KEY_W0;
    __IO uint32_t EXT_KEY_W1;
    __IO uint32_t EXT_KEY_W2;
    __IO uint32_t EXT_KEY_W3;
    __IO uint32_t EXT_KEY_W4;
    __IO uint32_t EXT_KEY_W5;
    __IO uint32_t EXT_KEY_W6;
    __IO uint32_t EXT_KEY_W7;
} AES_ACC_TypeDef;


/**************** Bit definition for AES_ACC_COMMAND register *****************/
#define AES_ACC_COMMAND_START_Pos       (0U)
#define AES_ACC_COMMAND_START_Msk       (0x1UL << AES_ACC_COMMAND_START_Pos)
#define AES_ACC_COMMAND_START           AES_ACC_COMMAND_START_Msk
#define AES_ACC_COMMAND_AES_ACC_RESET_Pos  (1U)
#define AES_ACC_COMMAND_AES_ACC_RESET_Msk  (0x1UL << AES_ACC_COMMAND_AES_ACC_RESET_Pos)
#define AES_ACC_COMMAND_AES_ACC_RESET   AES_ACC_COMMAND_AES_ACC_RESET_Msk
#define AES_ACC_COMMAND_AUTO_GATE_Pos   (2U)
#define AES_ACC_COMMAND_AUTO_GATE_Msk   (0x1UL << AES_ACC_COMMAND_AUTO_GATE_Pos)
#define AES_ACC_COMMAND_AUTO_GATE       AES_ACC_COMMAND_AUTO_GATE_Msk

/***************** Bit definition for AES_ACC_STATUS register *****************/
#define AES_ACC_STATUS_BUSY_Pos         (0U)
#define AES_ACC_STATUS_BUSY_Msk         (0x1UL << AES_ACC_STATUS_BUSY_Pos)
#define AES_ACC_STATUS_BUSY             AES_ACC_STATUS_BUSY_Msk
#define AES_ACC_STATUS_FLASH_KEY_VALID_Pos  (1U)
#define AES_ACC_STATUS_FLASH_KEY_VALID_Msk  (0x1UL << AES_ACC_STATUS_FLASH_KEY_VALID_Pos)
#define AES_ACC_STATUS_FLASH_KEY_VALID  AES_ACC_STATUS_FLASH_KEY_VALID_Msk

/****************** Bit definition for AES_ACC_IRQ register *******************/
#define AES_ACC_IRQ_DONE_STAT_Pos       (0U)
#define AES_ACC_IRQ_DONE_STAT_Msk       (0x1UL << AES_ACC_IRQ_DONE_STAT_Pos)
#define AES_ACC_IRQ_DONE_STAT           AES_ACC_IRQ_DONE_STAT_Msk
#define AES_ACC_IRQ_BUS_ERR_STAT_Pos    (1U)
#define AES_ACC_IRQ_BUS_ERR_STAT_Msk    (0x1UL << AES_ACC_IRQ_BUS_ERR_STAT_Pos)
#define AES_ACC_IRQ_BUS_ERR_STAT        AES_ACC_IRQ_BUS_ERR_STAT_Msk
#define AES_ACC_IRQ_SETUP_ERR_STAT_Pos  (2U)
#define AES_ACC_IRQ_SETUP_ERR_STAT_Msk  (0x1UL << AES_ACC_IRQ_SETUP_ERR_STAT_Pos)
#define AES_ACC_IRQ_SETUP_ERR_STAT      AES_ACC_IRQ_SETUP_ERR_STAT_Msk
#define AES_ACC_IRQ_DONE_RAW_STAT_Pos   (16U)
#define AES_ACC_IRQ_DONE_RAW_STAT_Msk   (0x1UL << AES_ACC_IRQ_DONE_RAW_STAT_Pos)
#define AES_ACC_IRQ_DONE_RAW_STAT       AES_ACC_IRQ_DONE_RAW_STAT_Msk
#define AES_ACC_IRQ_BUS_ERR_RAW_STAT_Pos  (17U)
#define AES_ACC_IRQ_BUS_ERR_RAW_STAT_Msk  (0x1UL << AES_ACC_IRQ_BUS_ERR_RAW_STAT_Pos)
#define AES_ACC_IRQ_BUS_ERR_RAW_STAT    AES_ACC_IRQ_BUS_ERR_RAW_STAT_Msk
#define AES_ACC_IRQ_SETUP_ERR_RAW_STAT_Pos  (18U)
#define AES_ACC_IRQ_SETUP_ERR_RAW_STAT_Msk  (0x1UL << AES_ACC_IRQ_SETUP_ERR_RAW_STAT_Pos)
#define AES_ACC_IRQ_SETUP_ERR_RAW_STAT  AES_ACC_IRQ_SETUP_ERR_RAW_STAT_Msk

/**************** Bit definition for AES_ACC_SETTING register *****************/
#define AES_ACC_SETTING_DONE_IRQ_MASK_Pos  (0U)
#define AES_ACC_SETTING_DONE_IRQ_MASK_Msk  (0x1UL << AES_ACC_SETTING_DONE_IRQ_MASK_Pos)
#define AES_ACC_SETTING_DONE_IRQ_MASK   AES_ACC_SETTING_DONE_IRQ_MASK_Msk
#define AES_ACC_SETTING_BUS_ERR_IRQ_MASK_Pos  (1U)
#define AES_ACC_SETTING_BUS_ERR_IRQ_MASK_Msk  (0x1UL << AES_ACC_SETTING_BUS_ERR_IRQ_MASK_Pos)
#define AES_ACC_SETTING_BUS_ERR_IRQ_MASK  AES_ACC_SETTING_BUS_ERR_IRQ_MASK_Msk
#define AES_ACC_SETTING_SETUP_ERR_IRQ_MASK_Pos  (2U)
#define AES_ACC_SETTING_SETUP_ERR_IRQ_MASK_Msk  (0x1UL << AES_ACC_SETTING_SETUP_ERR_IRQ_MASK_Pos)
#define AES_ACC_SETTING_SETUP_ERR_IRQ_MASK  AES_ACC_SETTING_SETUP_ERR_IRQ_MASK_Msk

/************** Bit definition for AES_ACC_AES_SETTING register ***************/
#define AES_ACC_AES_SETTING_AES_MODE_Pos  (0U)
#define AES_ACC_AES_SETTING_AES_MODE_Msk  (0x7UL << AES_ACC_AES_SETTING_AES_MODE_Pos)
#define AES_ACC_AES_SETTING_AES_MODE    AES_ACC_AES_SETTING_AES_MODE_Msk
#define AES_ACC_AES_SETTING_AES_LENGTH_Pos  (3U)
#define AES_ACC_AES_SETTING_AES_LENGTH_Msk  (0x3UL << AES_ACC_AES_SETTING_AES_LENGTH_Pos)
#define AES_ACC_AES_SETTING_AES_LENGTH  AES_ACC_AES_SETTING_AES_LENGTH_Msk
#define AES_ACC_AES_SETTING_KEY_SEL_Pos  (5U)
#define AES_ACC_AES_SETTING_KEY_SEL_Msk  (0x1UL << AES_ACC_AES_SETTING_KEY_SEL_Pos)
#define AES_ACC_AES_SETTING_KEY_SEL     AES_ACC_AES_SETTING_KEY_SEL_Msk
#define AES_ACC_AES_SETTING_ALGO_STANDARD_Pos  (6U)
#define AES_ACC_AES_SETTING_ALGO_STANDARD_Msk  (0x1UL << AES_ACC_AES_SETTING_ALGO_STANDARD_Pos)
#define AES_ACC_AES_SETTING_ALGO_STANDARD  AES_ACC_AES_SETTING_ALGO_STANDARD_Msk
#define AES_ACC_AES_SETTING_AES_OP_MODE_Pos  (7U)
#define AES_ACC_AES_SETTING_AES_OP_MODE_Msk  (0x1UL << AES_ACC_AES_SETTING_AES_OP_MODE_Pos)
#define AES_ACC_AES_SETTING_AES_OP_MODE  AES_ACC_AES_SETTING_AES_OP_MODE_Msk
#define AES_ACC_AES_SETTING_AES_BYPASS_Pos  (8U)
#define AES_ACC_AES_SETTING_AES_BYPASS_Msk  (0x1UL << AES_ACC_AES_SETTING_AES_BYPASS_Pos)
#define AES_ACC_AES_SETTING_AES_BYPASS  AES_ACC_AES_SETTING_AES_BYPASS_Msk

/***************** Bit definition for AES_ACC_DMA_IN register *****************/
#define AES_ACC_DMA_IN_ADDR_Pos         (0U)
#define AES_ACC_DMA_IN_ADDR_Msk         (0xFFFFFFFFUL << AES_ACC_DMA_IN_ADDR_Pos)
#define AES_ACC_DMA_IN_ADDR             AES_ACC_DMA_IN_ADDR_Msk

/**************** Bit definition for AES_ACC_DMA_OUT register *****************/
#define AES_ACC_DMA_OUT_ADDR_Pos        (0U)
#define AES_ACC_DMA_OUT_ADDR_Msk        (0xFFFFFFFFUL << AES_ACC_DMA_OUT_ADDR_Pos)
#define AES_ACC_DMA_OUT_ADDR            AES_ACC_DMA_OUT_ADDR_Msk

/**************** Bit definition for AES_ACC_DMA_DATA register ****************/
#define AES_ACC_DMA_DATA_SIZE_Pos       (0U)
#define AES_ACC_DMA_DATA_SIZE_Msk       (0xFFFFFFFUL << AES_ACC_DMA_DATA_SIZE_Pos)
#define AES_ACC_DMA_DATA_SIZE           AES_ACC_DMA_DATA_SIZE_Msk

/***************** Bit definition for AES_ACC_IV_W0 register ******************/
#define AES_ACC_IV_W0_DATA_Pos          (0U)
#define AES_ACC_IV_W0_DATA_Msk          (0xFFFFFFFFUL << AES_ACC_IV_W0_DATA_Pos)
#define AES_ACC_IV_W0_DATA              AES_ACC_IV_W0_DATA_Msk

/***************** Bit definition for AES_ACC_IV_W1 register ******************/
#define AES_ACC_IV_W1_DATA_Pos          (0U)
#define AES_ACC_IV_W1_DATA_Msk          (0xFFFFFFFFUL << AES_ACC_IV_W1_DATA_Pos)
#define AES_ACC_IV_W1_DATA              AES_ACC_IV_W1_DATA_Msk

/***************** Bit definition for AES_ACC_IV_W2 register ******************/
#define AES_ACC_IV_W2_DATA_Pos          (0U)
#define AES_ACC_IV_W2_DATA_Msk          (0xFFFFFFFFUL << AES_ACC_IV_W2_DATA_Pos)
#define AES_ACC_IV_W2_DATA              AES_ACC_IV_W2_DATA_Msk

/***************** Bit definition for AES_ACC_IV_W3 register ******************/
#define AES_ACC_IV_W3_DATA_Pos          (0U)
#define AES_ACC_IV_W3_DATA_Msk          (0xFFFFFFFFUL << AES_ACC_IV_W3_DATA_Pos)
#define AES_ACC_IV_W3_DATA              AES_ACC_IV_W3_DATA_Msk

/*************** Bit definition for AES_ACC_EXT_KEY_W0 register ***************/
#define AES_ACC_EXT_KEY_W0_DATA_Pos     (0U)
#define AES_ACC_EXT_KEY_W0_DATA_Msk     (0xFFFFFFFFUL << AES_ACC_EXT_KEY_W0_DATA_Pos)
#define AES_ACC_EXT_KEY_W0_DATA         AES_ACC_EXT_KEY_W0_DATA_Msk

/*************** Bit definition for AES_ACC_EXT_KEY_W1 register ***************/
#define AES_ACC_EXT_KEY_W1_DATA_Pos     (0U)
#define AES_ACC_EXT_KEY_W1_DATA_Msk     (0xFFFFFFFFUL << AES_ACC_EXT_KEY_W1_DATA_Pos)
#define AES_ACC_EXT_KEY_W1_DATA         AES_ACC_EXT_KEY_W1_DATA_Msk

/*************** Bit definition for AES_ACC_EXT_KEY_W2 register ***************/
#define AES_ACC_EXT_KEY_W2_DATA_Pos     (0U)
#define AES_ACC_EXT_KEY_W2_DATA_Msk     (0xFFFFFFFFUL << AES_ACC_EXT_KEY_W2_DATA_Pos)
#define AES_ACC_EXT_KEY_W2_DATA         AES_ACC_EXT_KEY_W2_DATA_Msk

/*************** Bit definition for AES_ACC_EXT_KEY_W3 register ***************/
#define AES_ACC_EXT_KEY_W3_DATA_Pos     (0U)
#define AES_ACC_EXT_KEY_W3_DATA_Msk     (0xFFFFFFFFUL << AES_ACC_EXT_KEY_W3_DATA_Pos)
#define AES_ACC_EXT_KEY_W3_DATA         AES_ACC_EXT_KEY_W3_DATA_Msk

/*************** Bit definition for AES_ACC_EXT_KEY_W4 register ***************/
#define AES_ACC_EXT_KEY_W4_DATA_Pos     (0U)
#define AES_ACC_EXT_KEY_W4_DATA_Msk     (0xFFFFFFFFUL << AES_ACC_EXT_KEY_W4_DATA_Pos)
#define AES_ACC_EXT_KEY_W4_DATA         AES_ACC_EXT_KEY_W4_DATA_Msk

/*************** Bit definition for AES_ACC_EXT_KEY_W5 register ***************/
#define AES_ACC_EXT_KEY_W5_DATA_Pos     (0U)
#define AES_ACC_EXT_KEY_W5_DATA_Msk     (0xFFFFFFFFUL << AES_ACC_EXT_KEY_W5_DATA_Pos)
#define AES_ACC_EXT_KEY_W5_DATA         AES_ACC_EXT_KEY_W5_DATA_Msk

/*************** Bit definition for AES_ACC_EXT_KEY_W6 register ***************/
#define AES_ACC_EXT_KEY_W6_DATA_Pos     (0U)
#define AES_ACC_EXT_KEY_W6_DATA_Msk     (0xFFFFFFFFUL << AES_ACC_EXT_KEY_W6_DATA_Pos)
#define AES_ACC_EXT_KEY_W6_DATA         AES_ACC_EXT_KEY_W6_DATA_Msk

/*************** Bit definition for AES_ACC_EXT_KEY_W7 register ***************/
#define AES_ACC_EXT_KEY_W7_DATA_Pos     (0U)
#define AES_ACC_EXT_KEY_W7_DATA_Msk     (0xFFFFFFFFUL << AES_ACC_EXT_KEY_W7_DATA_Pos)
#define AES_ACC_EXT_KEY_W7_DATA         AES_ACC_EXT_KEY_W7_DATA_Msk

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
