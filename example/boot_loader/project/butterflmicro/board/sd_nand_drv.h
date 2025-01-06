#ifndef _SD_NAND_DRV_H_
#define _SD_NAND_DRV_H_

#include <bf0_hal.h>
#include <string.h>
#include <stdio.h>
//#include <drv_io.h>


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

void sd1_init();
uint8_t sd1_send_cmd(uint8_t cmd_idx, uint32_t cmd_arg);
uint8_t sd1_send_acmd(uint8_t cmd_idx, uint32_t cmd_arg, uint16_t rca);
uint8_t sd1_wait_cmd();
void sd1_get_rsp(uint8_t *rsp_idx, uint32_t *rsp_arg1, uint32_t *rsp_arg2, uint32_t *rsp_arg3, uint32_t *rsp_arg4);
void sd1_read(uint8_t wire_mode, uint8_t block_num);
uint8_t sd1_wait_read();


uint8_t sdmmc1_sdnand();
int sd_read_data(uint32_t addr, uint8_t *data, uint32_t len);

int sdio_emmc_init();
int emmc_read_data(uint32_t addr, uint8_t *data, uint32_t len);

#endif  //_SD_NAND_DRV_H_
