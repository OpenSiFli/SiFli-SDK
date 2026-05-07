/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "sd_nand_drv.h"

#ifdef JLINK_SDEMMC_1
static uint32_t sdemmc_cache[128];
extern void debug_print(char *str);
extern uint8_t *htoa(uint8_t *p, uint32_t d);
static uint8_t  wire_mode = 0;    //0 for 1-wire mode, 1 for 4-wire mode

int sdmmc1_sdnand()
{
    int i;
    uint8_t  rsp_idx;
    uint32_t rsp_arg[4];
    uint8_t  cmd_result;
    uint32_t cmd_arg;
    uint8_t  ccs;
    uint16_t rca;
    uint32_t cid[4];
    uint32_t *buf;
    uint8_t hex[16];

    debug_print("SDMMC1 sd detect start!\n");

    //initialize sdmmc host
    sd1_init();
    hwp_sdmmc1->CLKCR = 119 << SD_CLKCR_DIV_Pos; //48M/120=400k
    hwp_sdmmc1->CLKCR |= SD_CLKCR_VOID_FIFO_ERROR;
    hwp_sdmmc1->IER = 0; //mask sdmmc interrupt
    hwp_sdmmc1->TOR = 0x02000000; //mask sdmmc interrupt

    // add a delay after clock set, at least 74 SD clock
    // need wait more than 200ms for 400khz
    HAL_Delay_us(500);
    rca = 0x0;

    debug_print("controller init done\n");

    //initialize sd card
    cmd_result = sd1_send_cmd(0,0); //CMD0

    //set sd_req and wait for sd_busy before access sd in normal mode
    hwp_sdmmc1->CASR = SD_CASR_SD_REQ;
    while ((hwp_sdmmc1->CASR & SD_CASR_SD_BUSY) == 0);
    debug_print("CMD 0 done\n");

    hwp_sdmmc1->CDR |= SD_CDR_CMD_OD;   // SET TO Open Drain mode

    //start card identification
    // CMD1
    do
    {
        cmd_arg = 0x40000080;
        cmd_result = sd1_send_cmd(1, cmd_arg); //CMD1

        sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
        debug_print("CMD1 resp0 0x");
        debug_print((char *)htoa(hex, rsp_arg[0]));
        debug_print(" \n");
        HAL_Delay_us(20);
    }
    while (!(rsp_arg[0] & 0x80000000));
    debug_print("CMD 1 done\n");

    //CMD2
    HAL_Delay_us(20);
    cmd_arg = 0x0;
    cmd_result = sd1_send_cmd(2, cmd_arg); //CMD2
    if (cmd_result == SD_TIMEOUT)
    {
        debug_print("CMD2 TIMEOUT!\n");
        return 1;
    }
    else if (cmd_result == SD_CRCERR)
    {
        debug_print("CMD2 SD_CRCERR!\n");
        return 2;
    }
    sd1_get_rsp(&rsp_idx, &cid[3], &cid[2], &cid[1], &cid[0]);
    debug_print("CMD 2 done\n");

    //CMD3
    HAL_Delay_us(20);
    rca = 1;
    cmd_arg = 0x10000;
    cmd_result = sd1_send_cmd(3, cmd_arg); //CMD3
    if (cmd_result == SD_TIMEOUT)
    {
        debug_print("CMD3 TIMEOUT!\n");
        return 3;
    }
    else if (cmd_result == SD_CRCERR)
    {
        debug_print("CMD3 SD_CRCERR!\n");
        return 4;
    }
    sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
    if (rsp_idx != 0x3)
    {
        debug_print("CMD3 RESONSE ERR!\n");
        return 5;
    }
    debug_print("CMD 3 done\n");
    hwp_sdmmc1->CDR &= ~SD_CDR_CMD_OD;  // recover to push pull mode

    HAL_Delay_us(20);
    cmd_arg = rca << 16;
    //cmd_arg = 0x10000;
    cmd_result = sd1_send_cmd(9, cmd_arg); //CMD9
    if (cmd_result == SD_TIMEOUT)
    {
        debug_print("CMD9 TIMEOUT!\n");
        return 6;
    }
    else if (cmd_result == SD_CRCERR)
    {
        debug_print("CMD9 CRC ERR!\n");
        return 7;
    }
    sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
    debug_print("CMD 9 done\n");
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

    debug_print("SD card identification done!\n");

    hwp_sdmmc1->CLKCR = 7 << SD_CLKCR_DIV_Pos; //48M/8=6M
    hwp_sdmmc1->CLKCR |= SD_CLKCR_VOID_FIFO_ERROR;
    hwp_sdmmc1->TOR = 0x02000000; // set timeout 
    hwp_sdmmc1->CDR = SD_CDR_ITIMING_SEL | (0<<SD_CDR_ITIMING_Pos);
    debug_print("Swtch to high clock\n");

    //start card transfer
    //CMD7 (SELECT_CARD)
    HAL_Delay_us(20);
    cmd_arg = (uint32_t)rca << 16;
    cmd_result = sd1_send_cmd(7, cmd_arg);
    if (cmd_result == SD_TIMEOUT)
    {
        debug_print("CMD7 TIMEOUT!\n");
        return 8;
    }
    else if (cmd_result == SD_CRCERR)
    {
        debug_print("CMD7 CRC ERR!\n");
        return 9;
    }
    sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
    if (rsp_idx != 7)
    {
        debug_print("CMD7 RSP ERR!\n");
        return 10;
    }
    debug_print("CMD 7 done\n");

    //CMD8 EXT_CSD
    hwp_sdmmc1->SR = 0xffffffff; //clear sdmmc interrupts
    sd1_read(0, 1); //1 wire mode,1 blocks
    cmd_arg = 0;
    cmd_result = sd1_send_cmd(8, cmd_arg);
    if (cmd_result == SD_TIMEOUT)
    {
        debug_print("cmd 8 timeout\n");
        return 11;
    }
    else if (cmd_result == SD_CRCERR)
    {
        debug_print("cmd 8 CRC ERROR\n");
        return 12;
    }
    sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
    if (rsp_idx != 8)
    {
        debug_print("CMD REP IDX NOT 8\n");
        return 13;
    }
    debug_print("CMD 8 done\n");

    //wait for dma read data
    hwp_sdmmc1->IER = SD_IER_DATA_DONE_MASK;
    cmd_result = sd1_wait_read();  //wait sdmmc interrupt
    if (hwp_sdmmc1->SR & SD_SR_DATA_TIMEOUT)
    {
        debug_print("Wait data timeout\n");
        return 13;
    }
    if (hwp_sdmmc1->SR & SD_SR_DATA_CRC)
    {
        debug_print("DATA CRC ERROR\n");
        return 14;
    }
    debug_print("CMD 8 data done\n");

    for (i = 0; i < 512 / 4; i++)
    {
        sdemmc_cache[i] = hwp_sdmmc1->FIFO;
    }
    // parse CSD
    for (i = 0; i < 512 / 4; i++)
    {
        debug_print(" 0x");
        debug_print((char *)htoa(hex, sdemmc_cache[i]));
        if ((i & 7) == 7)
            debug_print("\n");
    }
    // get cap
    uint32_t capacity = sdemmc_cache[212 / 4];
    capacity = capacity >> 10;
    capacity = capacity * 512;
    debug_print("Card Capacity = ");
    debug_print((char *)htoa(hex, capacity));
    debug_print(" KB \n");

#if 0
    //ACMD6
    HAL_Delay_us(20);
    uint32_t cmd6_index = 183;  // swtich line
    uint32_t cmd6_value = 1;
    cmd_arg = 0x03000000 | (cmd6_index << 16) | (cmd6_value << 8);
    cmd_arg = cmd_arg | 1;
    cmd_result = sd1_send_cmd(6, cmd_arg); //
    if (cmd_result == SD_TIMEOUT)
    {
      debug_print("ACMD6 TIMEOUT!\n");
      return 15;
    }
    else if (cmd_result == SD_CRCERR)
    {
      debug_print("ACMD6 CRC ERR!\n");
      return 16;
    }
    debug_print("CMD 6 done\n");

    sd1_read(wire_mode, 1); //4 wire mode,1 blocks
#else 
    sd1_read(0, 1); //1 wire mode,1 blocks
#endif
    //CMD17 (READ_SINGLE_BLOCK)
    HAL_Delay_us(20);
    hwp_sdmmc1->SR = 0xffffffff; //clear sdmmc interrupts
    cmd_arg = 0; //start data address
    cmd_result = sd1_send_cmd(17, cmd_arg);
    if (cmd_result == SD_TIMEOUT)
    {
      debug_print("CMD17 TIMEOUT!\n");
      return 17; 
    }
    else if (cmd_result == SD_CRCERR)
    {
      debug_print("CMD17 CRC ERR!\n");
      return 18;
    }
    sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
    if (rsp_idx != 17)
    {
      debug_print("CMD17 RSP ERR!\n");
      return 19;
    }
    debug_print("CMD 17 done\n");

    //wait read data
    hwp_sdmmc1->IER = SD_IER_DATA_DONE_MASK;
    cmd_result = sd1_wait_read();  //wait sdmmc interrupt
    if (hwp_sdmmc1->SR & SD_SR_DATA_TIMEOUT)
    {
      debug_print("DATA READ TIMEOUT!\n");
      return 20; // read time Out
    }
    if (hwp_sdmmc1->SR & SD_SR_DATA_CRC)
    {
      debug_print("READ CRC ERR!\n");
      return 21; // Data error
    }
    debug_print("CMD 17 data done\n");
    // read data
    for (i = 0; i < 512 / 4; i++)
    {
      sdemmc_cache[i] = hwp_sdmmc1->FIFO;
    }

    for (i = 0; i < 512 / 4; i++)
    {
        debug_print(" 0x");
        debug_print((char *)htoa(hex, sdemmc_cache[i]));
        if ((i & 7) == 7)
            debug_print("\n");
    }

    debug_print("SDMMC 1 initial finish\n");
  return 0;

}

int sd_read_data(uint32_t addr, uint8_t *data, uint32_t len)
{
    uint8_t  rsp_idx;
    uint8_t test_result = TEST_PASS;
    int i;
    uint32_t cmd_result;
    uint32_t cmd_arg;
    uint32_t *buf = (uint32_t *)data;
    uint32_t rsp_arg1,rsp_arg2,rsp_arg3,rsp_arg4;
    uint8_t hex[16];
    //debug_print("sd_read_data : Addr-");
    //debug_print((char *)htoa(hex, addr));
    //debug_print(" Size-");  
    //debug_print((char *)htoa(hex, len));
    //debug_print("\r\n"); 
      //dma config for read
      //hwp_dmac1->CPAR5  = &hwp_sdmmc1->FIFO;
      //hwp_dmac1->CM0AR5 = dma_dst_addr;
      //hwp_dmac1->CNDTR5 = 128;
      //hwp_dmac1->CSELR2 |= (57 << DMAC_CSELR2_C5S_Pos);
      //hwp_dmac1->CCR5 = DMAC_CCR5_MINC;
      //hwp_dmac1->CCR5 |= (0x2 << DMAC_CCR5_MSIZE_Pos) | (0x2 << DMAC_CCR5_PSIZE_Pos);
      //hwp_dmac1->CCR5 |= DMAC_CCR5_EN;
      //hwp_sdmmc1->FDLR = 64 | SD_FDLR_RD_VALID_EN; //dma request threshold is 128 word (one block)
    
      //start data read before read command
      sd1_read(wire_mode,1); //4 wire mode,8 blocks
    
      //CMD18 (READ_SINGLE_BLOCK)
    HAL_Delay_us(20);
    hwp_sdmmc1->SR = 0xffffffff; //clear sdmmc interrupts
    cmd_arg =  addr >> 9; //start data address
      cmd_result = sd1_send_cmd(17,cmd_arg);
      if (cmd_result == SD_TIMEOUT) {
        debug_print("CMD17 TIMEOUT!\n");
        test_result = TEST_FAIL;
      }
      else if (cmd_result == SD_CRCERR) {
        debug_print("CMD17 CRC ERR!\n");
        test_result = TEST_FAIL;
      }
      sd1_get_rsp(&rsp_idx,&rsp_arg1,&rsp_arg2,&rsp_arg3,&rsp_arg4);
      if (rsp_idx != 17) {
        debug_print("CMD17 RSP ERR!\n");
        test_result = TEST_FAIL;
      }
    
      //wait for dma read data
      hwp_sdmmc1->SR = 0xffffffff; //clear sdmmc interrupts
      hwp_sdmmc1->IER = SD_IER_DATA_DONE_MASK;
      cmd_result = sd1_wait_read();  //wait sdmmc interrupt
      if (hwp_sdmmc1->SR & SD_SR_DATA_TIMEOUT) {
        debug_print("DATA READ TIMEOUT!\n");
        test_result = TEST_FAIL;
      }
      if (hwp_sdmmc1->SR & SD_SR_DATA_CRC) {
        debug_print("READ CRC ERR!\n");
        test_result = TEST_FAIL;
      }

      for(i=0; i<len/4; i++)
      {
        *buf = hwp_sdmmc1->FIFO;
        buf++;
      }

    if(test_result == TEST_FAIL)
    {
        debug_print("Read page fail!\n");
        return 0;
    }

    return len;
}

int sd_write_data(uint32_t addr, uint8_t *data, uint32_t len)
{
    uint8_t  rsp_idx;
    uint8_t test_result = TEST_PASS;
    int i;
    uint32_t cmd_result;
    uint32_t cmd_arg;
    uint32_t rsp_arg1,rsp_arg2,rsp_arg3,rsp_arg4;

    uint32_t *buf = (uint32_t *)data;

    uint8_t hex[16];
    //debug_print("sd_write_data : Addr-");
    //debug_print((char *)htoa(hex, addr));
    //debug_print(" Size-");  
    //debug_print((char *)htoa(hex, len));
    //debug_print("\r\n"); 
    
    //CMD25 (WRITE_SINGLE_BLOCK)
    hwp_sdmmc1->SR = 0xffffffff; //clear sdmmc interrupts
    cmd_arg = addr >> 9; //start data address
    cmd_result = sd1_send_cmd(24,cmd_arg);
    if (cmd_result == SD_TIMEOUT) {
      debug_print("CMD24 TIMEOUT!\n");
      test_result = TEST_FAIL;
    }
    else if (cmd_result == SD_CRCERR) {
      debug_print("CMD24 CRC ERR!\n");
      test_result = TEST_FAIL;
    }
    sd1_get_rsp(&rsp_idx,&rsp_arg1,&rsp_arg2,&rsp_arg3,&rsp_arg4);
    if (rsp_idx != 24) {
      debug_print("CMD24 RSP ERR!\n");
      test_result = TEST_FAIL;
    }
      
    //start dma for write
    sd1_write(wire_mode,1); //4 wire mode,1 block
    hwp_sdmmc1->SR = 0xffffffff; //clear sdmmc interrupts
    hwp_sdmmc1->IER = SD_IER_DATA_DONE_MASK;

    for(i=0; i<len/4; i++)
    {
        hwp_sdmmc1->FIFO = *buf;
        buf++;
    }

    cmd_result = sd1_wait_write();  //wait sdmmc interrupt
    if (hwp_sdmmc1->SR & SD_SR_DATA_TIMEOUT) {
      debug_print("data write TIMEOUT!\n");
      test_result = TEST_FAIL;
    }

    if(test_result == TEST_FAIL)
    {
        debug_print("Write page fail\n");
        return 0;
    }

    return len;
}
#endif

