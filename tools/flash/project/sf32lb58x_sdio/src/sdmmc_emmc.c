/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <bf0_hal.h>
#include <string.h>
#include <stdio.h>
#include <drv_io.h>
#include <stdlib.h>
#include "sdmmc_tst_drv.h"

uint32_t sdmmc_base = SDMMC1_BASE;
static uint8_t sdemmc_cache[512];
extern void debug_print(char *str);
extern uint8_t *htoa(uint8_t *p, uint32_t d);
static uint8_t  wire_mode = 2;
#define RT_EOK 0


struct dfs_partition
{
    uint8_t type;        /* file system type */
    uint32_t  offset;       /* partition start offset */
    size_t size;         /* partition size */
    //rt_sem_t lock;
};
int dfs_filesystem_get_partition(struct dfs_partition *part, uint8_t *buf, uint32_t pindex);

uint8_t sdmmc_emmc()
{
    uint8_t test_result = TEST_UNFINISHED;
    uint8_t  rsp_idx;
    uint32_t rsp_arg1;
    uint32_t rsp_arg2;
    uint32_t rsp_arg3;
    uint32_t rsp_arg4;
    uint8_t  cmd_result;
    uint32_t cmd_arg;
    uint16_t i;
    uint8_t  ccs;
    uint16_t rca;
    uint32_t cid[4];
    uint32_t csd[4];
    uint32_t ext_csd[128];
    //uint8_t  wire_mode=2; //0 for 1-wire mode, 1 for 4-wire mode, 2 for 8-wire mode
    uint32_t device_status;
    uint8_t  cmd6_index, cmd6_value;
    uint32_t *databuf;

    uint8_t hex[16];

    //debug_print("sdmmc emmc init\r\n");

    //initialize sdmmc host
    //while(!(hwp_hpsys_aon->ACR & HPSYS_AON_ACR_HXT48_RDY_Msk));
    //hwp_hpsys_rcc->CSR |= 1 << HPSYS_RCC_CSR_SEL_SYS_Pos; //select hxt48
    //hwp_hpsys_rcc->ENR2 |= HPSYS_RCC_ENR2_SDMMC1;
    //debug_print("sdmmc clk en\r\n");
    sd_init();

    sd_wr_byte(SDHCI_POWER_CONTROL, 0x00); //open-drain mode for cmd line
    hwp_pinmux1->PAD_PA11 = 0x210;   // reset sdmmc
    for (i = 0; i < 1000; i++) {} //add some delay
    hwp_pinmux1->PAD_PA11 = 0x230;   // release reset sdmmc
    sd_wr_hword(SDHCI_CLOCK_CONTROL, (64 << SDHCI_DIVIDER_SHIFT) | SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_CARD_EN);
    rca = 0x1;

    //debug_print("before cmd 0\r\n");
    //reset eMMC
    cmd_result = sd_send_cmd(0, 0); //CMD0
    sd_clr_status();

    //start card identification
    //step 1, CMD8
    //cmd_arg = 0x000001aa; //VHS=1
    //cmd_result = sd_send_cmd(8,cmd_arg); //CMD8
    //if (cmd_result != SDHCI_INT_RESPONSE) {
    //  printf("CMD8 status error: %x!\n",cmd_result);
    //  test_result = TEST_FAIL;
    //}
    //sd_clr_status();
    //sd_get_rsp(&rsp_arg1,&rsp_arg2,&rsp_arg3,&rsp_arg4);
    //if (rsp_arg1 != 0x1aa) {
    //  printf("CMD8 RSP ERR!\n");
    //  test_result = TEST_FAIL;
    //}
    //debug_print("before cmd 1\r\n");

    //CMD1
    cmd_arg = 0x40000080;
    while (1)  //wait for card busy status
    {
        cmd_result = sd_send_cmd(1, cmd_arg);
        sd_clr_status();
        sd_get_rsp(&rsp_arg1, &rsp_arg2, &rsp_arg3, &rsp_arg4);
        if (rsp_arg1 == 0xC0FF8080)
        {
            break; //card power up done
        }
        for (i = 0; i < 1000; i++) {} //add some delay
    }
    //debug_print("before cmd 2\r\n");

    //CMD2
    cmd_arg = 0x0;
    cmd_result = sd_send_cmd(2, cmd_arg); //CMD2
    sd_clr_status();
    sd_get_rsp(&cid[0], &cid[1], &cid[2], &cid[3]);

    //CMD3
    cmd_arg = rca << 16;
    cmd_result = sd_send_cmd(3, cmd_arg); //CMD3
    sd_clr_status();
    sd_get_rsp(&rsp_arg1, &rsp_arg2, &rsp_arg3, &rsp_arg4);
    device_status = rsp_arg1;
    //card identification done

    sd_wr_byte(SDHCI_POWER_CONTROL, 0x10); //push-pull mode for cmd line

    //start card transfer
    //CMD9
    cmd_arg = (uint32_t)rca << 16;
    cmd_result = sd_send_cmd(9, cmd_arg);
    sd_clr_status();
    sd_get_rsp(&csd[0], &csd[1], &csd[2], &csd[3]);

    //CMD7 (SELECT_CARD)
    cmd_arg = (uint32_t)rca << 16;
    cmd_result = sd_send_cmd(7, cmd_arg);
    sd_clr_status();

    //CMD8 (SELECT_CARD)
    //sd_wr_hword(SDHCI_TRANSFER_MODE,SDHCI_TRNS_BLK_CNT_EN|SDHCI_TRNS_READ);
    //sd_read(1,1); //1 wire mode,single block
    //cmd_arg = 0x0;
    //cmd_result = sd_send_cmd(8,cmd_arg);
    //sd_clr_status();
    //sd_wait_read();
    //for (i=0;i<128;i++) {
    //  ext_csd[i] = sd_rd_word(SDHCI_BUFFER);
    //}
    //sd_wait_read();
    //sd_clr_status();


    //CMD6
    //write high speed mode
    cmd6_index = 185;
    cmd6_value = 0x01; //1 for high speed, 2 for hs200
    cmd_arg = 0x03000000 | (cmd6_index << 16) | (cmd6_value << 8);
    cmd_result = sd_send_cmd(6, cmd_arg);
    sd_clr_status();
    cmd6_index = 183;
    cmd6_value = wire_mode; //0 for sdr 1-wire, 1 for sdr 4-wire, 2 for sdr 8-wire
    cmd_arg = 0x03000000 | (cmd6_index << 16) | (cmd6_value << 8);
    cmd_result = sd_send_cmd(6, cmd_arg);
    sd_clr_status();

    //switch to fast clock
    sd_wr_hword(SDHCI_CLOCK_CONTROL, (0 << SDHCI_DIVIDER_SHIFT) | SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_CARD_EN);
    sd_wr_byte(SDHCI_HOST_CONTROL2, 0xb); //1.8V hs200
    sd_wr_byte(SDHCI_CLK_TUNE, 0x15); //set clock delay for hs200
    for (i = 0; i < 1000; i++) {} //add some delay


    //CMD17 (READ_SINGLE_BLOCK)
    sd_clr_status();
    sd_wr_hword(SDHCI_TRANSFER_MODE, SDHCI_TRNS_BLK_CNT_EN | SDHCI_TRNS_READ);
    sd_read(wire_mode, 1); //4 wire mode,single block
    cmd_arg = 0;// 0x100000>>9; //read first page
    cmd_result = sd_send_cmd(17, cmd_arg);
    if (cmd_result != SDHCI_INT_RESPONSE)
    {
        //debug_print((char *)htoa(hex, cmd_result));
        //debug_print("  CMD17 status error!\n");
        test_result = TEST_FAIL;
    }
    //start data read
    cmd_result = sd_wait_read();
    if (cmd_result != SDHCI_INT_DATA_AVAIL)
    {
        //debug_print((char *)htoa(hex, cmd_result));
        //debug_print(" read valid status error\n");
        test_result = TEST_FAIL;
    }
    sd_clr_status();

    //read data
    databuf = (uint32_t *)sdemmc_cache;
    for (i = 0; i < 128; i++)
    {
        *(databuf + i) = sd_rd_word(SDHCI_BUFFER);
    }

    //check transfer complete flag
    cmd_result = sd_wait_read();
    if (cmd_result != SDHCI_INT_DATA_END)
    {
        //debug_print((char *)htoa(hex, cmd_result));
        //debug_print(" read end status error!\n");
        test_result = TEST_FAIL;
    }
    sd_clr_status();

#if 0
    debug_print("data: 0x");
    for (i = 0; i < 4; i++)
    {
        debug_print((char *)htoa(hex, *(databuf + i)));
        debug_print(" 0x");
    }
    debug_print("\r\n");

    struct dfs_partition pts;
    for (i = 0; i < 16; i++)
    {
        dfs_filesystem_get_partition(&pts, sdemmc_cache, i);
    }
#endif

    if (test_result != TEST_FAIL)
    {
        debug_print("sdmmc init pass\r\n");
        test_result = TEST_PASS;
    }
    else
    {
        debug_print("sdmmc init fail\r\n");
    }
    //debug_print("sdmmc init done\r\n");

    return test_result;
}

int sd_read_data(uint32_t addr, uint8_t *data, uint32_t len)
{
    int i;
    uint32_t cmd_result;
    uint32_t cmd_arg;
    uint8_t test_result = TEST_PASS;
    uint32_t *buf = (uint32_t *)data;

    uint8_t hex[16];
    //debug_print("sd_read_data : Addr-");
    //debug_print((char *)htoa(hex, addr));
    //debug_print(" Size-");
    //debug_print((char *)htoa(hex, len));
    //debug_print("\r\n");

    //CMD17 (READ_SINGLE_BLOCK)
    sd_clr_status();
    sd_wr_hword(SDHCI_TRANSFER_MODE, SDHCI_TRNS_BLK_CNT_EN | SDHCI_TRNS_READ);
    sd_read(wire_mode, 1); //4 wire mode,single block
    cmd_arg = addr >> 9; //start data address
    cmd_result = sd_send_cmd(17, cmd_arg);
    if (cmd_result != SDHCI_INT_RESPONSE)
    {
        debug_print((char *)htoa(hex, cmd_result));
        debug_print("  CMD17 status error!\n");
        test_result = TEST_FAIL;
    }
    //start data read
    cmd_result = sd_wait_read();
    if (cmd_result != SDHCI_INT_DATA_AVAIL)
    {
        debug_print((char *)htoa(hex, cmd_result));
        debug_print(" read valid status error\n");
        test_result = TEST_FAIL;
    }
    sd_clr_status();

    //compare data
    for (i = 0; i < len / 4; i++)
    {
        *buf = sd_rd_word(SDHCI_BUFFER);
        buf++;
    }

    //check transfer complete flag
    cmd_result = sd_wait_read();
    if (cmd_result != SDHCI_INT_DATA_END)
    {
        debug_print((char *)htoa(hex, cmd_result));
        debug_print(" read end status error!\n");
        test_result = TEST_FAIL;
    }
    sd_clr_status();
#if 0
    buf = (uint32_t *)data;
    debug_print("data: 0x");
    for (i = 0; i < 4; i++)
    {
        debug_print((char *)htoa(hex, *(buf + i)));
        debug_print(" 0x");
    }
    debug_print("\r\n");
#endif

    if (test_result == TEST_FAIL)
    {
        debug_print("Read page fail!\n");
        return 0;
    }

    return len;
}

int sd_write_data(uint32_t addr, uint8_t *data, uint32_t len)
{
    int i;
    uint32_t cmd_result;
    uint32_t cmd_arg;
    uint8_t test_result = TEST_PASS;
    uint32_t *buf = (uint32_t *)data;

    uint8_t hex[16];
    //debug_print("sd_write_data : Addr-");
    //debug_print((char *)htoa(hex, addr));
    //debug_print(" Size-");
    //debug_print((char *)htoa(hex, len));
    //debug_print("\r\n");

    //CMD24 (WRITE_BLOCK)
    sd_clr_status();
    sd_wr_hword(SDHCI_TRANSFER_MODE, SDHCI_TRNS_BLK_CNT_EN);
    sd_write(wire_mode, 1); //4 wire mode,single block
    cmd_arg = addr >> 9; //start data address
    cmd_result = sd_send_cmd(24, cmd_arg);
    if (cmd_result != SDHCI_INT_RESPONSE)
    {
        debug_print((char *)htoa(hex, cmd_result));
        debug_print("  CMD24 status error!\n");
    }
    sd_clr_status();

    //write data
    for (i = 0; i < 128; i++)
    {
        sd_wr_word(SDHCI_BUFFER, *buf);
        buf++;
    }
    cmd_result = sd_wait_write();
    if (cmd_result != SDHCI_INT_DATA_END)
    {
        debug_print((char *)htoa(hex, cmd_result));
        debug_print(" write status error!\n");
        test_result = TEST_FAIL;
    }
    sd_clr_status();
    //printf("SD card write done!\n");

    if (test_result == TEST_FAIL)
    {
        debug_print("Write page fail\n");
        return 0;
    }

    return len;
}


int dfs_filesystem_get_partition(struct dfs_partition *part,
                                 uint8_t         *buf,
                                 uint32_t        pindex)
{
#define DPT_ADDRESS     0x1be       /* device partition offset in Boot Sector */
#define DPT_ITEM_SIZE   16          /* partition item size */

    uint8_t *dpt;
    uint8_t type;
    uint8_t hex[16];


    dpt = buf + DPT_ADDRESS + pindex * DPT_ITEM_SIZE;

    /* check if it is a valid partition table */
    if ((*dpt != 0x80) && (*dpt != 0x00))
        return -5;

    /* get partition type */
    type = *(dpt + 4);
    if (type == 0)
        return -5;

    /* set partition information
     *    size is the number of 512-Byte */
    part->type = type;
    part->offset = *(dpt + 8) | *(dpt + 9) << 8 | *(dpt + 10) << 16 | *(dpt + 11) << 24;
    part->size = *(dpt + 12) | *(dpt + 13) << 8 | *(dpt + 14) << 16 | *(dpt + 15) << 24;

    //rt_kprintf("found part[%d], begin: %d, size: ",
    //pindex, part->offset * 512);
    debug_print("found part[0x");
    debug_print((char *)htoa(hex, pindex));
    debug_print("],begin: ");
    debug_print((char *)htoa(hex, part->offset * 512));
    debug_print(", size: ");

    if ((part->size >> 11) == 0)
    {
        debug_print((char *)htoa(hex, part->size >> 1));
        debug_print("KB\r\n");
        //rt_kprintf("%d%s", part->size >> 1, "KB\n"); /* KB */
    }
    else
    {
        unsigned int part_size;
        part_size = part->size >> 11;                /* MB */
        if ((part_size >> 10) == 0)
        {
            debug_print((char *)htoa(hex, part_size));
            debug_print(".");
            debug_print((char *)htoa(hex, (part->size >> 1) & 0x3FF));
            debug_print("MB\r\n");
            //rt_kprintf("%d.%d%s", part_size, (part->size >> 1) & 0x3FF, "MB\n");
        }
        else
        {
            debug_print((char *)htoa(hex, part_size >> 10));
            debug_print(".");
            debug_print((char *)htoa(hex, part_size & 0x3FF));
            debug_print("GB\r\n");
            //rt_kprintf("%d.%d%s", part_size >> 10, part_size & 0x3FF, "GB\n");
        }
    }

    return RT_EOK;
}


