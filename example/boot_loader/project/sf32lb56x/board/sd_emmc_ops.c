#include <bf0_hal.h>
#include <string.h>
#include <stdio.h>
#include <drv_io.h>
#include <stdlib.h>
#include "sd_emmc_drv.h"

static uint8_t sdemmc_cache[512];
static uint8_t  wire_mode = 2;


struct dfs_partition
{
    uint8_t type;        /* file system type */
    uint32_t  offset;       /* partition start offset */
    size_t size;         /* partition size */
    //rt_sem_t lock;
};
int dfs_filesystem_get_partition(struct dfs_partition *part, uint8_t *buf, uint32_t pindex);

int sdmmc_init()
{
    uint32_t rsp_arg1;
    uint32_t rsp_arg2;
    uint32_t rsp_arg3;
    uint32_t rsp_arg4;
    uint8_t  cmd_result;
    uint32_t cmd_arg;
    uint16_t i;
    uint16_t rca;
    uint32_t cid[4];
    uint32_t csd[4];
    uint32_t device_status;
    uint8_t  cmd6_index, cmd6_value;
    uint32_t *databuf;

    //initialize sdmmc host
    //while(!(hwp_hpsys_aon->ACR & HPSYS_AON_ACR_HXT48_RDY_Msk));
    //hwp_hpsys_rcc->CSR |= 1 << HPSYS_RCC_CSR_SEL_SYS_Pos; //select hxt48
    //hwp_hpsys_rcc->ENR2 |= HPSYS_RCC_ENR2_SDMMC1;
    sd_init(SDMMC1_BASE);

    sd_wr_byte(SDHCI_POWER_CONTROL, 0x00); //open-drain mode for cmd line
    //hwp_pinmux1->PAD_PA11 = 0x210;   // reset sdmmc
    //for (i=0;i<1000;i++) {} //add some delay
    //hwp_pinmux1->PAD_PA11 = 0x230;   // release reset sdmmc
    sd_wr_hword(SDHCI_CLOCK_CONTROL, (255 << SDHCI_DIVIDER_SHIFT) | SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_CARD_EN);
    rca = 0x1;

    //reset eMMC
    cmd_result = sd_send_cmd(0, 0); //CMD0
    sd_clr_status();

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
    sd_wr_hword(SDHCI_CLOCK_CONTROL, (10 << SDHCI_DIVIDER_SHIFT) | SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_CARD_EN);
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
        //test_result = TEST_FAIL;
        ///while(1);
        HAL_ASSERT(0);
        return 1;
    }
    //start data read
    cmd_result = sd_wait_read();
    if (cmd_result != SDHCI_INT_DATA_AVAIL)
    {
        //test_result = TEST_FAIL;
        HAL_ASSERT(0);
        //while(1);
        return 2;
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
        //test_result = TEST_FAIL;
        HAL_ASSERT(0);
        //while(1);
        return 3;
    }
    sd_clr_status();

#if 0
    rt_kprintf("data: ");
    for (i = 0; i < 4; i++)
    {
        rt_kprintf(" 0x%08x", *(databuf + i));
    }
    rt_kprintf("\r\n");

    struct dfs_partition pts;
    for (i = 0; i < 16; i++)
    {
        dfs_filesystem_get_partition(&pts, sdemmc_cache, i);
    }
#endif

    return 0;
}

int sd_read_data(uint32_t addr, uint8_t *data, uint32_t len)
{
    int i;
    uint32_t cmd_result;
    uint32_t cmd_arg;
    uint32_t *buf = (uint32_t *)data;

    //CMD17 (READ_SINGLE_BLOCK)
    sd_clr_status();
    sd_wr_hword(SDHCI_TRANSFER_MODE, SDHCI_TRNS_BLK_CNT_EN | SDHCI_TRNS_READ);
    sd_read(wire_mode, 1); //4 wire mode,single block
    cmd_arg = addr >> 9; //start data address
    cmd_result = sd_send_cmd(17, cmd_arg);
    if (cmd_result != SDHCI_INT_RESPONSE)
    {
        //test_result = TEST_FAIL;
        return 0;
    }
    //start data read
    cmd_result = sd_wait_read();
    if (cmd_result != SDHCI_INT_DATA_AVAIL)
    {
        //test_result = TEST_FAIL;
        return 0;
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
        //test_result = TEST_FAIL;
        return 0;
    }
    sd_clr_status();
#if 0
    buf = (uint32_t *)data;
    rt_kprintf("data:  ");
    for (i = 0; i < 4; i++)
    {
        rt_kprintf(" 0x%08x", *(buf + i));
    }
    rt_kprintf("\r\n");
#endif

    return len;
}

#if 0
int sd_write_data(uint32_t addr, uint8_t *data, uint32_t len)
{
    int i;
    uint32_t cmd_result;
    uint32_t cmd_arg;
    uint32_t *buf = (uint32_t *)data;

    //CMD24 (WRITE_BLOCK)
    sd_clr_status();
    sd_wr_hword(SDHCI_TRANSFER_MODE, SDHCI_TRNS_BLK_CNT_EN);
    sd_write(wire_mode, 1); //4 wire mode,single block
    cmd_arg = addr >> 9; //start data address
    cmd_result = sd_send_cmd(24, cmd_arg);
    if (cmd_result != SDHCI_INT_RESPONSE)
    {
        //test_result = TEST_FAIL;
        return 0;
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
        //test_result = TEST_FAIL;
        return 0;
    }
    sd_clr_status();
    //rt_kprintf("SD card write done!\n");

    return len;
}
#endif

int dfs_filesystem_get_partition(struct dfs_partition *part,
                                 uint8_t         *buf,
                                 uint32_t        pindex)
{
#define DPT_ADDRESS     0x1be       /* device partition offset in Boot Sector */
#define DPT_ITEM_SIZE   16          /* partition item size */

    uint8_t *dpt;
    uint8_t type;


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
#if 0
    if ((part->size >> 11) == 0)
    {
        rt_kprintf("%d%s", part->size >> 1, "KB\n"); /* KB */
    }
    else
    {
        unsigned int part_size;
        part_size = part->size >> 11;                /* MB */
        if ((part_size >> 10) == 0)
        {
            rt_kprintf("%d.%d%s", part_size, (part->size >> 1) & 0x3FF, "MB\n");
        }
        else
        {
            rt_kprintf("%d.%d%s", part_size >> 10, part_size & 0x3FF, "GB\n");
        }
    }
#endif
    return 0;
}


