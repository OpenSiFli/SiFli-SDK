/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "sd_nand_drv.h"

#ifdef JLINK_SDIO_1
static uint8_t sdemmc_cache[512];
extern void debug_print(char *str);
extern uint8_t *htoa(uint8_t *p, uint32_t d);
static uint8_t  wire_mode=1;    //0 for 1-wire mode, 1 for 4-wire mode
static   uint8_t  sdsc = 1; //0 for sdhc/sdxc, 1 for sdsc

static uint8_t mmcsd_parse_csd(uint32_t *resp);
uint8_t sdmmc1_sdnand()
{
  uint8_t test_result = TEST_PASS;
  uint16_t i;
  uint8_t  rsp_idx;
  uint32_t rsp_arg[4];
  uint8_t  cmd_result;
  uint32_t cmd_arg;
  uint8_t  ccs;
  uint16_t rca;
  uint32_t cid[4];
  uint32_t *buf;
  uint8_t hex[16];
  //uint8_t  wire_mode = 1; 

  debug_print("SDMMC1 sd detect start!\n");

  //initialize sdmmc host
  sd1_init();
  hwp_sdmmc1->CLKCR = 119 << SD_CLKCR_DIV_Pos; //48M/120=400k
  hwp_sdmmc1->CLKCR |= SD_CLKCR_VOID_FIFO_ERROR;
  hwp_sdmmc1->IER = 0; //mask sdmmc interrupt
  hwp_sdmmc1->TOR = 0x02000000; //mask sdmmc interrupt
  rca = 0x0;

  //initialize sd card
  cmd_result = sd1_send_cmd(0,0); //CMD0

  //set sd_req and wait for sd_busy before access sd in normal mode
  hwp_sdmmc1->CASR = SD_CASR_SD_REQ;
  while ((hwp_sdmmc1->CASR & SD_CASR_SD_BUSY) == 0);

  //start card identification
  //CMD8
  cmd_arg = 0x000001aa; //VHS=1
  cmd_result = sd1_send_cmd(8,cmd_arg); //CMD8
  if (cmd_result == SD_TIMEOUT) {
    debug_print("CMD8 TIMEOUT!\n");
    test_result = TEST_FAIL;
  }
  else if (cmd_result == SD_CRCERR) {
    debug_print("CMD8 CRC ERR!\n");
    test_result = TEST_FAIL;
  }
  sd1_get_rsp(&rsp_idx,&rsp_arg[0],&rsp_arg[1],&rsp_arg[2],&rsp_arg[3]);
  if ((rsp_idx != 0x8) || (rsp_arg[0] != 0x1aa)) {
    debug_print("CMD8 RSP ERR!\n");
    test_result = TEST_FAIL;
  }

  //ACMD41
  cmd_arg = 0x40ff8000;
  while(1) { //wait for card busy status
    cmd_result = sd1_send_acmd(41,cmd_arg,rca); //CMD55+ACMD41
    if (cmd_result == SD_TIMEOUT) {
      debug_print("ACMD41 TIMEOUT!\n");
      test_result = TEST_FAIL;
    }
    sd1_get_rsp(&rsp_idx,&rsp_arg[0],&rsp_arg[1],&rsp_arg[2],&rsp_arg[3]);
    if ((rsp_arg[0] & 0x80000000) != 0) {
      break; //card power up done
    }
    for (i=0;i<1000;i++) {} //add some delay
	}
  ccs = (rsp_arg[0] >> 30) & 0x1;

  //CMD2
  cmd_arg = 0x0;
  cmd_result = sd1_send_cmd(2,cmd_arg); //CMD2
  if (cmd_result == SD_TIMEOUT) {
    debug_print("CMD2 TIMEOUT!\n");
    test_result = TEST_FAIL;
  }
  else if (cmd_result == SD_CRCERR) {
    debug_print("CMD2 CRC ERR!\n");
    test_result = TEST_FAIL;
  }  
  sd1_get_rsp(&rsp_idx,&cid[3],&cid[2],&cid[1],&cid[0]);
  //if ((rsp_idx != 0x3f) || (rsp_arg1 != 0x66450082) || (rsp_arg2 != 0x20100251) ||
  //                         (rsp_arg3 != 0x534d4920) || (rsp_arg4 != 0x001b534d)) {
  //  printf("CMD2 RSP ERR!\n");
  //  test_result = TEST_FAIL;
  //}

  //CMD3
  cmd_arg = 0x0;
  cmd_result = sd1_send_cmd(3,cmd_arg); //CMD3
  if (cmd_result == SD_TIMEOUT) {
    debug_print("CMD3 TIMEOUT!\n");
    test_result = TEST_FAIL;
  }
  else if (cmd_result == SD_CRCERR) {
    debug_print("CMD3 CRC ERR!\n");
    test_result = TEST_FAIL;
  }
  sd1_get_rsp(&rsp_idx,&rsp_arg[0],&rsp_arg[1],&rsp_arg[2],&rsp_arg[3]);
  rca = rsp_arg[0] >> 16;
  if (rsp_idx != 0x3) {
    debug_print("CMD3 RSP ERR!\n");
    test_result = TEST_FAIL;
  }

    cmd_arg = rca << 16;
    cmd_result = sd1_send_cmd(9,cmd_arg); //CMD9
    if (cmd_result == SD_TIMEOUT) {
      debug_print("CMD9 TIMEOUT!\n");
      test_result = TEST_FAIL;
    }
    else if (cmd_result == SD_CRCERR) {
      debug_print("CMD9 CRC ERR!\n");
      test_result = TEST_FAIL;
    }
    sd1_get_rsp(&rsp_idx,&rsp_arg[0],&rsp_arg[1],&rsp_arg[2],&rsp_arg[3]);
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
    if(csd_struct == 0)
    {
        debug_print("SDSC card!\n");
        sdsc = 1;
    }
    else if(csd_struct == 1)
    {
        debug_print("SDHC card!\n");
        sdsc = 0;
    }
    else
    {
        debug_print("SD card invalid csd structure !\n");
        test_result = TEST_FAIL;
    }

  debug_print("SD card identification done!\n");
	
  hwp_sdmmc1->CLKCR = 1 << SD_CLKCR_DIV_Pos; //48M/2=24M
  hwp_sdmmc1->CLKCR |= SD_CLKCR_VOID_FIFO_ERROR;
  hwp_sdmmc1->CDR = SD_CDR_ITIMING_SEL | (0<<SD_CDR_ITIMING_Pos);

  //start card transfer
  //CMD7 (SELECT_CARD)
  cmd_arg = (uint32_t)rca << 16;
  cmd_result = sd1_send_cmd(7,cmd_arg);
  if (cmd_result == SD_TIMEOUT) {
    debug_print("CMD7 TIMEOUT!\n");
    test_result = TEST_FAIL;
  }
  else if (cmd_result == SD_CRCERR) {
    debug_print("CMD7 CRC ERR!\n");
    test_result = TEST_FAIL;
  }
  sd1_get_rsp(&rsp_idx,&rsp_arg[0],&rsp_arg[1],&rsp_arg[2],&rsp_arg[3]);
  if (rsp_idx != 7) {
    debug_print("CMD7 RSP ERR!\n");
    test_result = TEST_FAIL;
  }

  //ACMD6
  cmd_arg = wire_mode ? 2 : 0; //select 4-wire mode or 1-wire mode
  cmd_result = sd1_send_acmd(6,cmd_arg,rca); //CMD55+ACMD6
  if (cmd_result == SD_TIMEOUT) {
    debug_print("ACMD6 TIMEOUT!\n");
    test_result = TEST_FAIL;
  }
  else if (cmd_result == SD_CRCERR) {
    debug_print("ACMD6 CRC ERR!\n");
    test_result = TEST_FAIL;
  }

  sd1_read(wire_mode,1); //4 wire mode,8 blocks
  
  //CMD17 (READ_SINGLE_BLOCK)
  cmd_arg = 0; //start data address
  cmd_result = sd1_send_cmd(17,cmd_arg);
  if (cmd_result == SD_TIMEOUT) {
    debug_print("CMD17 TIMEOUT!\n");
    test_result = TEST_FAIL;
  }
  else if (cmd_result == SD_CRCERR) {
    debug_print("CMD17 CRC ERR!\n");
    test_result = TEST_FAIL;
  }
  sd1_get_rsp(&rsp_idx, &rsp_arg[0],&rsp_arg[1],&rsp_arg[2],&rsp_arg[3]);
  if (rsp_idx != 17) {
    debug_print("CMD17 RSP ERR!\n");
    test_result = TEST_FAIL;
  }
  
  //wait read data
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

  buf = (uint32_t *)sdemmc_cache;
  for(i=0; i<512/4; i++)
  {
    *buf = hwp_sdmmc1->FIFO;
    buf++;
  }
#if 0
    debug_print("data:");
    for(i=0; i<8; i++)
    {
        debug_print(" 0x");
        debug_print((char *)htoa(hex, *(buf+i)));
    }
    debug_print("\r\n");

#endif

  if (test_result != TEST_FAIL) 
  {
    debug_print("SD1 init pass\n");
  }
  else 
  {
    debug_print("SD1 init fail\n");
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
      cmd_arg = sdsc ? addr : addr>>9; //start data address
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

#if 0
      debug_print("start ahb read!\n");
      //clear sd_busy to start ahb mode
      hwp_sdmmc1->CASR = SD_CASR_SD_BUSY;
    
      //select card type
      if (sdsc == 0) {
        hwp_sdmmc1->CACR &= ~SD_CACR_CACHE_SDSC; //for SDHC/SDXC
      }
        
      //set cache offset
      hwp_sdmmc1->CAOFF = 0;
#endif
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
    cmd_arg = sdsc ? addr : addr>>9; //start data address
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

struct mmcsd_csd {
    uint8_t		csd_structure;	/* CSD register version */
    uint8_t		taac;
    uint8_t		nsac;
    uint8_t		tran_speed;	/* max data transfer rate */
    uint16_t	card_cmd_class;	/* card command classes */
    uint8_t		rd_blk_len;	/* max read data block length */
    uint8_t		rd_blk_part;
    uint8_t		wr_blk_misalign;
    uint8_t		rd_blk_misalign;
    uint8_t		dsr_imp;	/* DSR implemented */
    uint8_t		c_size_mult;	/* CSD 1.0 , device size multiplier */
    uint32_t	c_size;		/* device size */
    uint8_t		r2w_factor;
    uint8_t		wr_blk_len;	/* max wtire data block length */
    uint8_t		wr_blk_partial;
    uint8_t		csd_crc;
};

#if 0
struct mmcsd_card 
{
    uint16_t    flags;
    uint16_t	tacc_clks;	/* data access time by ns */
    uint32_t	tacc_ns;	/* data access time by clk cycles */
    uint32_t	max_data_rate;	/* max data transfer rate */
    uint32_t	card_capacity;	/* card capacity, unit:KB */
    uint32_t	card_blksize;	/* card block size */
};

static const rt_uint32_t tran_unit[] =
{
    10000, 100000, 1000000, 10000000,
    0,     0,      0,       0
};

static const rt_uint8_t tran_value[] =
{
    0,  10, 12, 13, 15, 20, 25, 30,
    35, 40, 45, 50, 55, 60, 70, 80,
};
static const rt_uint32_t tacc_uint[] =
{
    1, 10, 100, 1000, 10000, 100000, 1000000, 10000000,
};

static const rt_uint8_t tacc_value[] =
{
    0,  10, 12, 13, 15, 20, 25, 30,
    35, 40, 45, 50, 55, 60, 70, 80,
};
#endif
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
    struct mmcsd_csd csd_st;
    struct mmcsd_csd *csd = &csd_st;

    csd->csd_structure = GET_BITS(resp, 126, 2);
    return csd->csd_structure;
#if 0    
    struct mmcsd_card card_st;
    struct mmcsd_card *card = &card_st;

    switch (csd->csd_structure)
    {
    case 0:
        csd->taac = GET_BITS(resp, 112, 8);
        csd->nsac = GET_BITS(resp, 104, 8);
        csd->tran_speed = GET_BITS(resp, 96, 8);
        csd->card_cmd_class = GET_BITS(resp, 84, 12);
        csd->rd_blk_len = GET_BITS(resp, 80, 4);
        csd->rd_blk_part = GET_BITS(resp, 79, 1);
        csd->wr_blk_misalign = GET_BITS(resp, 78, 1);
        csd->rd_blk_misalign = GET_BITS(resp, 77, 1);
        csd->dsr_imp = GET_BITS(resp, 76, 1);
        csd->c_size = GET_BITS(resp, 62, 12);
        csd->c_size_mult = GET_BITS(resp, 47, 3);
        csd->r2w_factor = GET_BITS(resp, 26, 3);
        csd->wr_blk_len = GET_BITS(resp, 22, 4);
        csd->wr_blk_partial = GET_BITS(resp, 21, 1);
        csd->csd_crc = GET_BITS(resp, 1, 7);

        card->card_blksize = 1 << csd->rd_blk_len;
        card->card_capacity = (csd->c_size + 1) << (csd->c_size_mult + 2);
        card->card_capacity *= card->card_blksize;
        card->card_capacity >>= 10; /* unit:KB */
        card->tacc_clks = csd->nsac * 100;
        card->tacc_ns = (tacc_uint[csd->taac & 0x07] * tacc_value[(csd->taac & 0x78) >> 3] + 9) / 10;
        card->max_data_rate = tran_unit[csd->tran_speed & 0x07] * tran_value[(csd->tran_speed & 0x78) >> 3];
        debug_print("CSD structure version 0\r\n");
        break;
    case 1:
        card->flags |= (1<<1);

        /*This field is fixed to 0Eh, which indicates 1 ms.
          The host should not use TAAC, NSAC, and R2W_FACTOR
          to calculate timeout and should uses fixed timeout
          values for read and write operations*/
        csd->taac = GET_BITS(resp, 112, 8);
        csd->nsac = GET_BITS(resp, 104, 8);
        csd->tran_speed = GET_BITS(resp, 96, 8);
        csd->card_cmd_class = GET_BITS(resp, 84, 12);
        csd->rd_blk_len = GET_BITS(resp, 80, 4);
        csd->rd_blk_part = GET_BITS(resp, 79, 1);
        csd->wr_blk_misalign = GET_BITS(resp, 78, 1);
        csd->rd_blk_misalign = GET_BITS(resp, 77, 1);
        csd->dsr_imp = GET_BITS(resp, 76, 1);
        csd->c_size = GET_BITS(resp, 48, 22);

        csd->r2w_factor = GET_BITS(resp, 26, 3);
        csd->wr_blk_len = GET_BITS(resp, 22, 4);
        csd->wr_blk_partial = GET_BITS(resp, 21, 1);
        csd->csd_crc = GET_BITS(resp, 1, 7);

        card->card_blksize = 512;
        card->card_capacity = (csd->c_size + 1) * 512;  /* unit:KB */
        card->tacc_clks = 0;
        card->tacc_ns = 0;
        card->max_data_rate = tran_unit[csd->tran_speed & 0x07] * tran_value[(csd->tran_speed & 0x78) >> 3];
        debug_print("CSD structure version 1\r\n");
        break;
    default:
        debug_print("unrecognised CSD structure version!");
    }

    return 0;
#endif
}

#endif
