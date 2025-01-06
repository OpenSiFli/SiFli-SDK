#include <bf0_hal.h>
#include <string.h>
#include <stdio.h>
#include <drv_io.h>

#include "sd_emmc_drv.h"


static uint32_t sdmmc_base;

__inline void sd_wr_word(uint8_t reg_addr, uint32_t data)
{
    write_memory(sdmmc_base + (uint32_t)reg_addr, data);
}

__inline void sd_wr_hword(uint8_t reg_addr, uint16_t data)
{
    write_hword(sdmmc_base + (uint32_t)reg_addr, data);
}

__inline void sd_wr_byte(uint8_t reg_addr, uint8_t data)
{
    write_byte(sdmmc_base + (uint32_t)reg_addr, data);
}

__inline uint32_t sd_rd_word(uint8_t reg_addr)
{
    return read_memory(sdmmc_base + (uint32_t)reg_addr);
}

__inline uint16_t sd_rd_hword(uint8_t reg_addr)
{
    return read_hword(sdmmc_base + (uint32_t)reg_addr);
}

__inline uint8_t sd_rd_byte(uint8_t reg_addr)
{
    return read_hword(sdmmc_base + (uint32_t)reg_addr);
}

void sd_init(uint32_t reg_base)
{
    sdmmc_base = reg_base;
    hwp_pmuc->HXT_CR1 |= PMUC_HXT_CR1_BUF_DLL_EN;
    //dll1 set to 192M
    //hwp_hpsys_rcc->DLL1CR = (hwp_hpsys_rcc->DLL1CR & ~HPSYS_RCC_DLL1CR_STG_Msk) |
    //                        (0x3 << HPSYS_RCC_DLL1CR_STG_Pos);
    //hwp_hpsys_rcc->DLL1CR = (hwp_hpsys_rcc->DLL1CR & ~HPSYS_RCC_DLL1CR_OUT_DIV2_EN_Msk) |
    //                        (0x0 << HPSYS_RCC_DLL1CR_OUT_DIV2_EN_Pos);
    //hwp_hpsys_rcc->DLL1CR = (hwp_hpsys_rcc->DLL1CR & ~HPSYS_RCC_DLL1CR_STG_Msk) |
    //                        (0x0 << HPSYS_RCC_DLL1CR_STG_Pos);
    //hwp_hpsys_rcc->DLL1CR |= HPSYS_RCC_DLL1CR_EN;
    //while (!(hwp_hpsys_rcc->DLL1CR & HPSYS_RCC_DLL1CR_READY_Msk));
    //hwp_hpsys_rcc->CFGR = (0x1 << HPSYS_RCC_CFGR_HDIV_Pos) |
    //                      (0x1 << HPSYS_RCC_CFGR_PDIV1_Pos)|
    //                      (0x5 << HPSYS_RCC_CFGR_PDIV2_Pos);
    //hwp_hpsys_rcc->CSR |= 1 << HPSYS_RCC_CSR_SEL_SYS_Pos; //select dll1
    //pinmux_sdmmc1(2);
    //pinmux_sdmmc2();
    //set sd base clock
    sd_wr_hword(SDHCI_CLOCK_CONTROL, SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_CARD_EN);
    //set timeout
    sd_wr_hword(SDHCI_TIMEOUT_CONTROL, 0xe);
    //enable status
    sd_wr_word(SDHCI_INT_ENABLE, SDHCI_INT_ALL_MASK);
    //enable all INCR for dma burst
    sd_wr_word(0xEC, 0xf);
}

uint32_t sd_send_cmd(uint8_t cmd_idx, uint32_t cmd_arg)
{
    enum resp resp_type;
    uint8_t with_data;
    uint8_t cmd_para;

    //write command argument
    sd_wr_word(SDHCI_ARGUMENT, cmd_arg);

    switch (cmd_idx)
    {
    case  0:
    case  4:
    case 15:
        resp_type = RESP_NONE;
        break;
    case  2:
    case  9:
    case 10:
        resp_type = RESP_R2;
        break;
    case 41:
        resp_type = RESP_R3;
        break; //ACMD41
    case  3:
        resp_type = RESP_R6;
        break;
    case  8:
        resp_type = RESP_R7;
        break;
    case  6:
    case  7:
        resp_type = RESP_R1b;
        break;
    default:
        resp_type = RESP_R1;
        break;
    }

    switch (cmd_idx)
    {
    case 24:
    case 25:
    case 21:
    case 17:
    case 18:
        with_data = 1;
        break;
    default:
        with_data = 0;
        break;
    }
    switch (resp_type)
    {
    case RESP_NONE:
        cmd_para = SDHCI_CMD_RESP_NONE;
        break;
    case RESP_R2:
        cmd_para = SDHCI_CMD_RESP_LONG | SDHCI_CMD_CRC;
        break;
    case RESP_R3:
    case RESP_R4:
        cmd_para = SDHCI_CMD_RESP_SHORT;
        break;
    case RESP_R1:
    case RESP_R5:
    case RESP_R6:
    case RESP_R7:
        cmd_para = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX;
        break;
    case RESP_R1b:
    case RESP_R5b:
        cmd_para = SDHCI_CMD_RESP_SHORT_BUSY | SDHCI_CMD_CRC | SDHCI_CMD_INDEX;
        break;
    default:
        cmd_para = SDHCI_CMD_RESP_SHORT;
        break;
    }

    if (with_data) cmd_para |= SDHCI_CMD_DATA;

    //write command register
    sd_wr_hword(SDHCI_COMMAND, SDHCI_MAKE_CMD(cmd_idx, cmd_para));

    return sd_wait_cmd();
}

uint32_t sd_send_acmd(uint8_t cmd_idx, uint32_t cmd_arg, uint16_t rca)
{
    uint8_t cmd_result;

    cmd_result = sd_send_cmd(55, (uint32_t)rca << 16);
    if (cmd_result != SDHCI_INT_RESPONSE)
        return cmd_result;
    sd_clr_status();

    cmd_result = sd_send_cmd(cmd_idx, cmd_arg);
    return cmd_result;
}

uint32_t sd_wait_cmd()
{
    while ((sd_rd_word(SDHCI_INT_STATUS) & SDHCI_INT_CMD_MASK) == 0);
    return (sd_rd_word(SDHCI_INT_STATUS) & SDHCI_INT_CMD_MASK);
}

uint32_t sd_clr_status()
{
    sd_wr_word(SDHCI_INT_STATUS, SDHCI_INT_ALL_MASK);
    return 0;
}

void sd_get_rsp(uint32_t *rsp_arg1, uint32_t *rsp_arg2, uint32_t *rsp_arg3, uint32_t *rsp_arg4)
{
    *rsp_arg1 = sd_rd_word(SDHCI_RESPONSE);
    *rsp_arg2 = sd_rd_word(SDHCI_RESPONSE + 0x4);
    *rsp_arg3 = sd_rd_word(SDHCI_RESPONSE + 0x8);
    *rsp_arg4 = sd_rd_word(SDHCI_RESPONSE + 0xc);
}

uint32_t sd_identify(uint16_t *rca)
{
    uint8_t  cmd_result;
    uint32_t cmd_arg;
    uint32_t rsp_arg1, rsp_arg2, rsp_arg3, rsp_arg4;

    //step 1, CMD8
    cmd_arg = 0x000001aa; //VHS=1
    cmd_result = sd_send_cmd(8, cmd_arg); //CMD8
    if (cmd_result != SDHCI_INT_RESPONSE) return cmd_result;
    sd_clr_status();
    //step 2, ACMD41
    cmd_result = sd_send_acmd(41, 0, 0); //CMD55+ACMD41
    if (cmd_result != SDHCI_INT_RESPONSE) return cmd_result;
    sd_clr_status();
    //step 3, CMD2
    cmd_result = sd_send_cmd(2, 0); //CMD2
    if (cmd_result != SDHCI_INT_RESPONSE) return cmd_result;
    sd_clr_status();
    //step 4, CMD3
    cmd_result = sd_send_cmd(3, 0); //CMD3
    if (cmd_result != SDHCI_INT_RESPONSE) return cmd_result;
    sd_clr_status();
    sd_get_rsp(&rsp_arg1, &rsp_arg2, &rsp_arg3, &rsp_arg4);
    *rca = rsp_arg1 >> 16;
    return cmd_result;
}

void sd_write(uint8_t wire_mode, uint8_t block_num)
{
    //sd_wr_byte(SDHCI_HOST_CONTROL,((wire_mode==2)?SDHCI_CTRL_8BITBUS:(wire_mode==1)?SDHCI_CTRL_4BITBUS:0)|SDHCI_CTRL_HISPD);
    sd_wr_byte(SDHCI_HOST_CONTROL, ((wire_mode == 2) ? SDHCI_CTRL_8BITBUS : (wire_mode == 1) ? SDHCI_CTRL_4BITBUS : 0));
    sd_wr_hword(SDHCI_BLOCK_SIZE, SDHCI_MAKE_BLKSZ(0, SD_BLOCK_SIZE));
    sd_wr_hword(SDHCI_BLOCK_COUNT, block_num);
}

uint32_t sd_wait_write()
{
    while ((sd_rd_word(SDHCI_INT_STATUS) & SDHCI_INT_DATA_MASK) == 0);
    return (sd_rd_word(SDHCI_INT_STATUS) & SDHCI_INT_DATA_MASK);
}

void sd_read(uint8_t wire_mode, uint8_t block_num)
{
    //sd_wr_byte(SDHCI_HOST_CONTROL,((wire_mode==2)?SDHCI_CTRL_8BITBUS:(wire_mode==1)?SDHCI_CTRL_4BITBUS:0)|SDHCI_CTRL_HISPD);
    sd_wr_byte(SDHCI_HOST_CONTROL, ((wire_mode == 2) ? SDHCI_CTRL_8BITBUS : (wire_mode == 1) ? SDHCI_CTRL_4BITBUS : 0));
    sd_wr_hword(SDHCI_BLOCK_SIZE, SDHCI_MAKE_BLKSZ(0, SD_BLOCK_SIZE));
    sd_wr_hword(SDHCI_BLOCK_COUNT, block_num);
}

uint32_t sd_wait_read()
{
    while ((sd_rd_word(SDHCI_INT_STATUS) & SDHCI_INT_DATA_MASK) == 0);
    return (sd_rd_word(SDHCI_INT_STATUS) & SDHCI_INT_DATA_MASK);
}


