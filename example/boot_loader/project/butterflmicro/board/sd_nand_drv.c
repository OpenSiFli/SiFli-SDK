
#include "sd_nand_drv.h"

void sd1_init()
{
    hwp_hpsys_rcc->ENR2 &= ~HPSYS_RCC_ENR2_SDMMC1;
    //pinmux_sdmmc1(1);
    HAL_Delay_us(100);
    hwp_hpsys_rcc->ENR2 |= HPSYS_RCC_ENR2_SDMMC1;
    hwp_hpsys_cfg->SYSCR |= HPSYS_CFG_SYSCR_SDNAND;
    hwp_sdmmc1->CLKCR = 0x1 << SD_CLKCR_DIV_Pos; //also clear sd_stop_clk
    hwp_sdmmc1->CDR = 0; //no card detect
    //hwp_sdmmc1->TOR = SD_BLOCK_SIZE*8*100;
}

uint8_t sd1_send_cmd(uint8_t cmd_idx, uint32_t cmd_arg)
{
    uint32_t ccr;
    uint8_t has_rsp;
    uint8_t long_rsp;

    hwp_sdmmc1->CAR = cmd_arg;
    switch (cmd_idx)
    {
    case  2:
    case  9:
    case 10:
        has_rsp = 1;
        long_rsp = 1;
        break;
    case  0:
    case  4:
    case 15:
        has_rsp = 0;
        long_rsp = 0;
        break;
    default:
        has_rsp = 1;
        long_rsp = 0;
        break;
    }
    ccr = (cmd_idx << SD_CCR_CMD_INDEX_Pos);
    if (has_rsp)
        ccr |= SD_CCR_CMD_HAS_RSP;
    if (long_rsp)
        ccr |= SD_CCR_CMD_LONG_RSP;
    ccr |= SD_CCR_CMD_TX_EN | SD_CCR_CMD_START;
    hwp_sdmmc1->CCR = ccr;
    return sd1_wait_cmd();
}

uint8_t sd1_send_acmd(uint8_t cmd_idx, uint32_t cmd_arg, uint16_t rca)
{
    uint32_t ccr;
    uint8_t cmd_result;

    cmd_result = sd1_send_cmd(55, (uint32_t)rca << 16);
    if (cmd_result != SD_SUCCESS)
        return cmd_result;
    hwp_sdmmc1->CAR = cmd_arg;
    ccr = (cmd_idx << SD_CCR_CMD_INDEX_Pos) | SD_CCR_CMD_HAS_RSP;
    ccr |= SD_CCR_CMD_TX_EN | SD_CCR_CMD_START;
    hwp_sdmmc1->CCR = ccr;
    cmd_result = sd1_wait_cmd();
    if ((cmd_result == SD_CRCERR) && (cmd_idx == 41))   //no CRC check for R3
    {
        hwp_sdmmc1->SR = SD_SR_CMD_RSP_CRC; //clear crc error status
        cmd_result = SD_SUCCESS;
    }
    return cmd_result;
}

uint8_t sd1_wait_cmd()
{
    while ((hwp_sdmmc1->SR & (SD_SR_CMD_DONE | SD_SR_CMD_TIMEOUT)) == 0);
    hwp_sdmmc1->SR = SD_SR_CMD_DONE; //clear cmd done status
    if (hwp_sdmmc1->SR & SD_SR_CMD_TIMEOUT)
        return SD_TIMEOUT;
    if (hwp_sdmmc1->SR & SD_SR_CMD_RSP_CRC)
        return SD_CRCERR;
    return SD_SUCCESS;
}

void sd1_get_rsp(uint8_t *rsp_idx, uint32_t *rsp_arg1, uint32_t *rsp_arg2, uint32_t *rsp_arg3, uint32_t *rsp_arg4)
{
    *rsp_idx = hwp_sdmmc1->RIR;
    *rsp_arg1 = hwp_sdmmc1->RAR1;
    *rsp_arg2 = hwp_sdmmc1->RAR2;
    *rsp_arg3 = hwp_sdmmc1->RAR3;
    *rsp_arg4 = hwp_sdmmc1->RAR4;
}

void sd1_write(uint8_t wire_mode, uint8_t block_num)
{
    uint32_t dcr;
    hwp_sdmmc1->DLR = (SD_BLOCK_SIZE * block_num) - 1;
    dcr = ((SD_BLOCK_SIZE - 1) << SD_DCR_BLOCK_SIZE_Pos);
    dcr |= (wire_mode << SD_DCR_WIRE_MODE_Pos);
    dcr |= SD_DCR_TRAN_DATA_EN | SD_DCR_DATA_START;
    hwp_sdmmc1->DCR = dcr;
}

uint8_t sd1_wait_write()
{
    while ((hwp_sdmmc1->SR & SD_SR_DATA_DONE) == 0);
    hwp_sdmmc1->SR = SD_SR_DATA_DONE; //clear cmd done status
    if (hwp_sdmmc1->SR & SD_SR_DATA_TIMEOUT)
        return SD_TIMEOUT;
    return SD_SUCCESS;
}

void sd1_read(uint8_t wire_mode, uint8_t block_num)
{
    uint32_t dcr;
    hwp_sdmmc1->DLR = (SD_BLOCK_SIZE * block_num) - 1;
    dcr = ((SD_BLOCK_SIZE - 1) << SD_DCR_BLOCK_SIZE_Pos);
    dcr |= (wire_mode << SD_DCR_WIRE_MODE_Pos) | SD_DCR_R_WN;
    dcr |= SD_DCR_TRAN_DATA_EN | SD_DCR_DATA_START;
    hwp_sdmmc1->DCR = dcr;
}

uint8_t sd1_wait_read()
{
    uint32_t mask = SD_SR_DATA_DONE | SD_SR_DATA_TIMEOUT | SD_SR_DATA_CRC;
    while ((hwp_sdmmc1->SR & mask) == 0);
    hwp_sdmmc1->SR = SD_SR_DATA_DONE; //clear cmd done status
    if (hwp_sdmmc1->SR & SD_SR_DATA_TIMEOUT)
        return SD_TIMEOUT;
    if (hwp_sdmmc1->SR & SD_SR_DATA_CRC)
        return SD_CRCERR;
    return SD_SUCCESS;
}

