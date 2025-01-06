#include <stdio.h>
#include "sd_nand_drv.h"

static uint8_t  wire_mode = 1;  //0 for 1-wire mode, 1 for 4-wire mode
static uint8_t  sdsc = 1; //0 for sdhc/sdxc, 1 for sdsc

static uint8_t mmcsd_parse_csd(uint32_t *resp);

uint8_t sdmmc1_sdnand()
{
    uint8_t test_result = 1;
    uint8_t  rsp_idx;
    uint32_t rsp_arg[4];
    uint8_t  cmd_result;
    uint8_t  ccs;
    uint16_t rca;
    uint32_t cmd_arg;
    uint32_t cid[4];

    //debug_print("SDMMC1 sd case start!\n");

    //initialize sdmmc host
    sd1_init();
#ifdef FPGA
    hwp_sdmmc1->CLKCR = 119 << SD_CLKCR_DIV_Pos; //48M/120=400k, stop_clk = 0
#else
    hwp_sdmmc1->CLKCR = 359 << SD_CLKCR_DIV_Pos; //144M/360=400k, stop_clk = 0
#endif
    hwp_sdmmc1->CLKCR |= SD_CLKCR_VOID_FIFO_ERROR;
    hwp_sdmmc1->IER = 0; //mask sdmmc interrupt
    hwp_sdmmc1->TOR = 0x00100000; // set timeout for 400K about 2.6s

    // add a delay after clock set, at least 74 SD clock
    // need wait more than 200ms for 400khz
    HAL_Delay_us(500);

    rca = 0x0;

    //initialize sd card
    cmd_result = sd1_send_cmd(0, 0); //CMD0

    //set sd_req and wait for sd_busy before access sd in normal mode
    hwp_sdmmc1->CASR = SD_CASR_SD_REQ;
    while ((hwp_sdmmc1->CASR & SD_CASR_SD_BUSY) == 0);

    //start card identification
    //CMD8
    HAL_Delay_us(20);
    cmd_arg = 0x000001aa; //VHS=1
    cmd_result = sd1_send_cmd(8, cmd_arg); //CMD8
    if (cmd_result == SD_TIMEOUT)
    {
        //debug_print("CMD8 TIMEOUT!\n");
        test_result = '8';
        //HAL_ASSERT(0);
        return test_result;
    }
    else if (cmd_result == SD_CRCERR)
    {
        //debug_print("CMD8 CRC ERR!\n");
        test_result = '8';
        //HAL_ASSERT(0);
        return test_result;
    }
    sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
    if ((rsp_idx != 0x8) || (rsp_arg[0] != 0x1aa))
    {
        //debug_print("CMD8 RSP ERR!\n");
        test_result = '8';
        //HAL_ASSERT(0);
        return test_result;
    }

    //ACMD41
    cmd_arg = 0x40ff8000;
    while (1) //wait for card busy status
    {
        HAL_Delay_us(20);
        cmd_result = sd1_send_acmd(41, cmd_arg, rca); //CMD55+ACMD41
        if (cmd_result == SD_TIMEOUT)
        {
            //debug_print("ACMD41 TIMEOUT!\n");
            test_result = '4';
            //HAL_ASSERT(0);
            return test_result; //CMD 41
        }
        sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
        if ((rsp_arg[0] & 0x80000000) != 0)
        {
            break; //card power up done
        }
        HAL_Delay_us(2); //add some delay
    }
    ccs = (rsp_arg[0] >> 30) & 0x1;

    //CMD2
    HAL_Delay_us(20);
    cmd_arg = 0x0;
    cmd_result = sd1_send_cmd(2, cmd_arg); //CMD2
    if (cmd_result == SD_TIMEOUT)
    {
        //debug_print("CMD2 TIMEOUT!\n");
        test_result = '2';
        //HAL_ASSERT(0);
        return test_result;
    }
    else if (cmd_result == SD_CRCERR)
    {
        //debug_print("CMD2 CRC ERR!\n");
        test_result = '2';
        //HAL_ASSERT(0);
        return test_result;
    }
    sd1_get_rsp(&rsp_idx, &cid[3], &cid[2], &cid[1], &cid[0]);

    //CMD3
    HAL_Delay_us(20);
    cmd_arg = 0x0;
    cmd_result = sd1_send_cmd(3, cmd_arg); //CMD3
    if (cmd_result == SD_TIMEOUT)
    {
        //debug_print("CMD3 TIMEOUT!\n");
        test_result = '3';
        //HAL_ASSERT(0);
        return test_result;
    }
    else if (cmd_result == SD_CRCERR)
    {
        //debug_print("CMD3 CRC ERR!\n");
        test_result = '3';
        //HAL_ASSERT(0);
        return test_result;
    }
    sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
    rca = rsp_arg[0] >> 16;
    if (rsp_idx != 0x3)
    {
        //debug_print("CMD3 RSP ERR!\n");
        test_result = '3';
        //HAL_ASSERT(0);
        return test_result;
    }

    HAL_Delay_us(20);
    cmd_arg = rca << 16;
    cmd_result = sd1_send_cmd(9, cmd_arg); //CMD9
    if (cmd_result == SD_TIMEOUT)
    {
        //debug_print("CMD9 TIMEOUT!\n");
        test_result = '9';
        //HAL_ASSERT(0);
        return test_result;
    }
    else if (cmd_result == SD_CRCERR)
    {
        //debug_print("CMD9 CRC ERR!\n");
        test_result = '9';
        //HAL_ASSERT(0);
        return test_result;
    }
    sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
    {
        // FOR R2, it need 128 bits response, high/low words should switch.
        // least 8 bit has been removed, so need fill 8 bits at least bits
        uint32_t temp;
        // switch for [0] as highest
        temp = rsp_arg[0];
        rsp_arg[0] = rsp_arg[3];
        rsp_arg[3] = temp;
        temp = rsp_arg[1];
        rsp_arg[1] = rsp_arg[2];
        rsp_arg[2] = temp;

        // << 8
        rsp_arg[0] = (rsp_arg[0] << 8) | (rsp_arg[1] >> 24);
        rsp_arg[1] = (rsp_arg[1] << 8) | (rsp_arg[2] >> 24);
        rsp_arg[2] = (rsp_arg[2] << 8) | (rsp_arg[3] >> 24);
        rsp_arg[3] = (rsp_arg[3] << 8);
    }

    uint8_t csd_struct = mmcsd_parse_csd(&rsp_arg[0]);
    if (csd_struct == 0)
    {
        //debug_print("SDSC card!\n");
        sdsc = 1;
    }
    else if (csd_struct == 1)
    {
        //debug_print("SDHC card!\n");
        sdsc = 0;
    }
    else
    {
        //debug_print("SD card invalid csd structure !\n");
        test_result = 'T';
        //HAL_ASSERT(0);
        return test_result; // structure Type fail
    }
    //debug_print("SD card identification done!\n");

#ifdef FPGA
    hwp_sdmmc1->CLKCR = 1 << SD_CLKCR_DIV_Pos; //48M/2=24M
#else
    hwp_sdmmc1->CLKCR = 5 << SD_CLKCR_DIV_Pos; //144M/6=24M
#endif
    hwp_sdmmc1->CLKCR |= SD_CLKCR_VOID_FIFO_ERROR;
    hwp_sdmmc1->TOR = 0x02000000; // set timeout for 24M about 1.4s
    hwp_sdmmc1->CDR = SD_CDR_ITIMING_SEL | (0 << SD_CDR_ITIMING_Pos);

    //start card transfer
    //CMD7 (SELECT_CARD)
    HAL_Delay_us(20);
    cmd_arg = (uint32_t)rca << 16;
    cmd_result = sd1_send_cmd(7, cmd_arg);
    if (cmd_result == SD_TIMEOUT)
    {
        //debug_print("CMD7 TIMEOUT!\n");
        test_result = '7';
        //HAL_ASSERT(0);
        return test_result;
    }
    else if (cmd_result == SD_CRCERR)
    {
        //debug_print("CMD7 CRC ERR!\n");
        test_result = '7';
        //HAL_ASSERT(0);
        return test_result;
    }
    sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
    if (rsp_idx != 7)
    {
        //debug_print("CMD7 RSP ERR!\n");
        test_result = '7';
        //HAL_ASSERT(0);
        return test_result;
    }

    //ACMD6
    HAL_Delay_us(20);
    cmd_arg = wire_mode ? 2 : 0; //select 4-wire mode or 1-wire mode
    cmd_result = sd1_send_acmd(6, cmd_arg, rca); //CMD55+ACMD6
    if (cmd_result == SD_TIMEOUT)
    {
        //debug_print("ACMD6 TIMEOUT!\n");
        test_result = '6';
        //HAL_ASSERT(0);
        return test_result;
    }
    else if (cmd_result == SD_CRCERR)
    {
        //debug_print("ACMD6 CRC ERR!\n");
        test_result = '6';
        //HAL_ASSERT(0);
        return test_result;
    }

    sd1_read(wire_mode, 1); //4 wire mode,8 blocks

    //CMD17 (READ_SINGLE_BLOCK)
    HAL_Delay_us(20);
    cmd_arg = 0; //start data address
    cmd_result = sd1_send_cmd(17, cmd_arg);
    if (cmd_result == SD_TIMEOUT)
    {
        //debug_print("CMD17 TIMEOUT!\n");
        test_result = 'R';
        //HAL_ASSERT(0);
        return test_result; // Read command
    }
    else if (cmd_result == SD_CRCERR)
    {
        //debug_print("CMD17 CRC ERR!\n");
        test_result = 'R';
        //HAL_ASSERT(0);
        return test_result;
    }
    sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
    if (rsp_idx != 17)
    {
        //debug_print("CMD17 RSP ERR!\n");
        test_result = 'R';
        //HAL_ASSERT(0);
        return test_result;
    }

    //wait read data
    hwp_sdmmc1->SR = 0xffffffff; //clear sdmmc interrupts
    hwp_sdmmc1->IER = SD_IER_DATA_DONE_MASK;
    cmd_result = sd1_wait_read();  //wait sdmmc interrupt
    if (hwp_sdmmc1->SR & SD_SR_DATA_TIMEOUT)
    {
        //debug_print("DATA READ TIMEOUT!\n");
        test_result = 'O';
        //HAL_ASSERT(0);
        return test_result; // read time Out
    }
    if (hwp_sdmmc1->SR & SD_SR_DATA_CRC)
    {
        //debug_print("READ CRC ERR!\n");
        test_result = 'D';
        //HAL_ASSERT(0);
        return test_result; // Data error
    }

    return test_result;
}

int sd_read_data(uint32_t addr, uint8_t *data, uint32_t len)
{
    uint8_t  rsp_idx;
    uint8_t test_result = TEST_PASS;
    int i;
    uint32_t cmd_result;
    uint32_t cmd_arg;
    uint32_t *buf = (uint32_t *)data;
    uint32_t rsp_arg1, rsp_arg2, rsp_arg3, rsp_arg4;

    //start data read before read command
    sd1_read(wire_mode, 1); //4 wire mode,1 blocks

    //CMD17 (READ_SINGLE_BLOCK)
    HAL_Delay_us(20);
    cmd_arg = sdsc ? addr : addr >> 9; //start data address
    cmd_result = sd1_send_cmd(17, cmd_arg);
    if (cmd_result == SD_TIMEOUT)
    {
        //debug_print("CMD17 TIMEOUT!\n");
        test_result = TEST_FAIL;
        //HAL_ASSERT(0);
        return 0;
    }
    else if (cmd_result == SD_CRCERR)
    {
        //debug_print("CMD17 CRC ERR!\n");
        test_result = TEST_FAIL;
        //HAL_ASSERT(0);
        return 0;
    }
    sd1_get_rsp(&rsp_idx, &rsp_arg1, &rsp_arg2, &rsp_arg3, &rsp_arg4);
    if (rsp_idx != 17)
    {
        //debug_print("CMD17 RSP ERR!\n");
        test_result = TEST_FAIL;
        //HAL_ASSERT(0);
        return 0;
    }

    //wait for dma read data
    hwp_sdmmc1->SR = 0xffffffff; //clear sdmmc interrupts
    hwp_sdmmc1->IER = SD_IER_DATA_DONE_MASK;
    cmd_result = sd1_wait_read();  //wait sdmmc interrupt
    if (hwp_sdmmc1->SR & SD_SR_DATA_TIMEOUT)
    {
        //debug_print("DATA READ TIMEOUT!\n");
        test_result = TEST_FAIL;
        //HAL_ASSERT(0);
        return 0;
    }
    if (hwp_sdmmc1->SR & SD_SR_DATA_CRC)
    {
        //debug_print("READ CRC ERR!\n");
        test_result = TEST_FAIL;
        //HAL_ASSERT(0);
        return 0;
    }

    for (i = 0; i < len / 4; i++)
    {
        *buf = hwp_sdmmc1->FIFO;
        buf++;
    }

    if (test_result == TEST_FAIL)
    {
        //debug_print("Read page fail!\n");
        return 0;
    }

    return len;
}

static __inline uint32_t GET_BITS(uint32_t *resp, uint32_t  start, uint32_t  size)
{
    const int32_t __size = size;
    const uint32_t __mask = (__size < 32 ? 1 << __size : 0) - 1;
    const int32_t __off = 3 - ((start) / 32);
    const int32_t __shft = (start) & 31;
    uint32_t __res;

    __res = resp[__off] >> __shft;
    if (__size + __shft > 32)
        __res |= resp[__off - 1] << ((32 - __shft) % 32);

    return __res & __mask;
}

static uint8_t mmcsd_parse_csd(uint32_t *resp)
{
    uint32_t csd_structure = GET_BITS(resp, 126, 2);
    return (uint8_t)csd_structure;
}

