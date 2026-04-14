/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SDMMC_DRV_H
#define SDMMC_DRV_H

#include "stdint.h"
#include <bf0_hal.h>
#include <string.h>
#include <stdio.h>

#define SD_SUCCESS 0
#define SD_TIMEOUT 1
#define SD_CRCERR  2

#define TEST_FAIL       0x0
#define TEST_PASS       0x1
#define TEST_UNFINISHED 0x2

#define SD_BLOCK_SIZE 512

#define SDIO_WR_ARG(F,A,D) (0x80000000 | (F<<28) | (A<<9) | D)
#define SDIO_RD_ARG(F,A) ((F<<28) | (A<<9))
#define SDIO_BWR_ARG(F,B,A,C) (0x80000000 | (F<<28) | (B<<27) | (A<<9) | C)
#define SDIO_BRD_ARG(F,B,A,C) ((F<<28) | (B<<27) | (A<<9) | C)

enum resp 
{
  RESP_NONE = 0,
  RESP_R1,
  RESP_R1b,
  RESP_R2,
  RESP_R3,
  RESP_R4,
  RESP_R5,
  RESP_R5b,
  RESP_R6,
  RESP_R7
};
#define read_memory(addr)        (*(volatile unsigned int *)((addr)))
#define write_memory(addr,value) (*(volatile unsigned int *)((addr))) = (value)
#define read_byte(addr)          (*(volatile unsigned char *)((addr)))
#define write_byte(addr,value)   (*(volatile unsigned char *)((addr))) = (value)
#define read_hword(addr)         (*(volatile unsigned short *)((addr)))
#define write_hword(addr,value)  (*(volatile unsigned short *)((addr))) = (value)

void sd_wr_word(uint8_t reg_addr,uint32_t data);
void sd_wr_hword(uint8_t reg_addr,uint16_t data);
void sd_wr_byte(uint8_t reg_addr,uint8_t data);
uint32_t sd_rd_word(uint8_t reg_addr);
uint16_t sd_rd_hword(uint8_t reg_addr);
uint8_t sd_rd_byte(uint8_t reg_addr);

void sd_init();
uint32_t sd_send_cmd(uint8_t cmd_idx, uint32_t cmd_arg);
uint32_t sd_send_acmd(uint8_t cmd_idx, uint32_t cmd_arg, uint16_t rca);
uint32_t sd_wait_cmd();
uint32_t sd_clr_status();
void sd_get_rsp(uint32_t *rsp_arg1, uint32_t *rsp_arg2, uint32_t *rsp_arg3, uint32_t *rsp_arg4);
uint32_t sd_identify(uint16_t *rca);
void sd_write(uint8_t wire_mode,uint8_t block_num);
uint32_t sd_wait_write();
void sd_read(uint8_t wire_mode,uint8_t block_num);
uint32_t sd_wait_read();
uint32_t sd_tuning_emmc(uint8_t wire_mode,uint64_t *tuning_err); 

void sd2_init(void);
uint8_t sd2_send_cmd(uint8_t cmd_idx, uint32_t cmd_arg);
uint8_t sd2_send_acmd(uint8_t cmd_idx, uint32_t cmd_arg, uint16_t rca);
uint8_t sd2_wait_cmd();
void sd2_get_rsp(uint8_t *rsp_idx, uint32_t *rsp_arg1, uint32_t *rsp_arg2, uint32_t *rsp_arg3, uint32_t *rsp_arg4);
uint8_t sd2_identify(uint16_t *rca);
void sd2_write(uint8_t wire_mode,uint8_t block_num);
uint8_t sd2_wait_write();
void sd2_read(uint8_t wire_mode,uint8_t block_num);
uint8_t sd2_wait_read();
uint8_t sd2_iowrite(uint8_t func,uint32_t addr,uint8_t data);
uint8_t sd2_ioread(uint8_t func,uint32_t addr,uint8_t *data);

#endif /* SDMMC_DRV_H */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/