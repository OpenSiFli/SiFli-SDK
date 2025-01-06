#ifndef SDMMC_TST_DRV_H
#define SDMMC_TST_DRV_H

#include "stdint.h"

#define SD_SUCCESS 0
#define SD_TIMEOUT 1
#define SD_CRCERR  2


#define SD_BLOCK_SIZE 512

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

void sd_wr_word(uint8_t reg_addr, uint32_t data);
void sd_wr_hword(uint8_t reg_addr, uint16_t data);
void sd_wr_byte(uint8_t reg_addr, uint8_t data);
uint32_t sd_rd_word(uint8_t reg_addr);
uint16_t sd_rd_hword(uint8_t reg_addr);
uint8_t sd_rd_byte(uint8_t reg_addr);

void sd_init(uint32_t reg_base);
uint32_t sd_send_cmd(uint8_t cmd_idx, uint32_t cmd_arg);
uint32_t sd_send_acmd(uint8_t cmd_idx, uint32_t cmd_arg, uint16_t rca);
uint32_t sd_wait_cmd();
uint32_t sd_clr_status();
void sd_get_rsp(uint32_t *rsp_arg1, uint32_t *rsp_arg2, uint32_t *rsp_arg3, uint32_t *rsp_arg4);
uint32_t sd_identify(uint16_t *rca);
void sd_write(uint8_t wire_mode, uint8_t block_num);
uint32_t sd_wait_write();
void sd_read(uint8_t wire_mode, uint8_t block_num);
uint32_t sd_wait_read();

int sdmmc_init();
int sd_read_data(uint32_t addr, uint8_t *data, uint32_t len);


#endif
