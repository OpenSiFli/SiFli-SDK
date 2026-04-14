/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "sd_nand_drv.h"

#define SD_USE_DMA  1

static uint8_t wire_mode = 1;  // 0: 1-wire mode, 1: 4-wire mode
static uint8_t sdsc = 1;       // 0: SDHC/SDXC, 1: SDSC

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

    // initialize sdmmc host
    sd1_init();
#ifdef FPGA
    hwp_sdmmc1->CLKCR = 119 << SD_CLKCR_DIV_Pos; // 48M/120=400k, stop_clk = 0
#else
    hwp_sdmmc1->CLKCR = 359 << SD_CLKCR_DIV_Pos; // 144M/360=400k, stop_clk = 0
#endif
    hwp_sdmmc1->CLKCR |= SD_CLKCR_VOID_FIFO_ERROR;
    hwp_sdmmc1->IER = 0; // mask sdmmc interrupt
    hwp_sdmmc1->TOR = 0x00100000; // timeout for 400K about 2.6s

    // add a delay after clock set: at least 74 SD clocks
    // need to wait >200ms for 400kHz
    HAL_Delay_us(500);

    rca = 0x0;

    // initialize sd card
    cmd_result = sd1_send_cmd(0, 0);  // CMD0

    // set sd_req and wait for sd_busy before accessing sd in normal mode
    hwp_sdmmc1->CASR = SD_CASR_SD_REQ;
    while ((hwp_sdmmc1->CASR & SD_CASR_SD_BUSY) == 0)
        ;

    // start card identification
    // CMD8
    HAL_Delay_us(20);
    cmd_arg = 0x000001aa;                     // VHS=1
    cmd_result = sd1_send_cmd(8, cmd_arg);    // CMD8
    if (cmd_result == SD_TIMEOUT)
    {
        test_result = '8';
        return test_result;
    }
    else if (cmd_result == SD_CRCERR)
    {
        test_result = '8';
        return test_result;
    }
    sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
    if ((rsp_idx != 0x8) || (rsp_arg[0] != 0x1aa))
    {
        test_result = '8';
        return test_result;
    }

    // ACMD41
    cmd_arg = 0x40ff8000;
    while (1)  // wait for card busy status
    {
        HAL_Delay_us(20);
        cmd_result = sd1_send_acmd(41, cmd_arg, rca);  // CMD55 + ACMD41
        if (cmd_result == SD_TIMEOUT)
        {
            test_result = '4';
            return test_result;  // CMD 41
        }
        sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
        if ((rsp_arg[0] & 0x80000000) != 0)
        {
            break;  // card power up done
        }
        HAL_Delay_us(2);  // add small delay
    }
    ccs = (rsp_arg[0] >> 30) & 0x1;

    // CMD2
    HAL_Delay_us(20);
    cmd_arg = 0x0;
    cmd_result = sd1_send_cmd(2, cmd_arg); // CMD2
    if (cmd_result == SD_TIMEOUT)
    {
        test_result = '2';
        return test_result;
    }
    else if (cmd_result == SD_CRCERR)
    {
        test_result = '2';
        return test_result;
    }
    sd1_get_rsp(&rsp_idx, &cid[3], &cid[2], &cid[1], &cid[0]);

    // CMD3
    HAL_Delay_us(20);
    cmd_arg = 0x0;
    cmd_result = sd1_send_cmd(3, cmd_arg); // CMD3
    if (cmd_result == SD_TIMEOUT)
    {
        test_result = '3';
        return test_result;
    }
    else if (cmd_result == SD_CRCERR)
    {
        test_result = '3';
        return test_result;
    }
    sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
    rca = rsp_arg[0] >> 16;
    if (rsp_idx != 0x3)
    {
        test_result = '3';
        return test_result;
    }

    // CMD9
    HAL_Delay_us(20);
    cmd_arg = rca << 16;
    cmd_result = sd1_send_cmd(9, cmd_arg); // CMD9
    if (cmd_result == SD_TIMEOUT)
    {
        test_result = '9';
        return test_result;
    }
    else if (cmd_result == SD_CRCERR)
    {
        test_result = '9';
        return test_result;
    }
    sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
    {
        // For R2: 128-bit response, swap high/low words; lowest 8 bits removed, so fill back 8 bits.
        uint32_t temp;
        // switch for [0] as highest
        temp = rsp_arg[0];
        rsp_arg[0] = rsp_arg[3];
        rsp_arg[3] = temp;
        temp = rsp_arg[1];
        rsp_arg[1] = rsp_arg[2];
        rsp_arg[2] = temp;

        // shift left by 8
        rsp_arg[0] = (rsp_arg[0] << 8) | (rsp_arg[1] >> 24);
        rsp_arg[1] = (rsp_arg[1] << 8) | (rsp_arg[2] >> 24);
        rsp_arg[2] = (rsp_arg[2] << 8) | (rsp_arg[3] >> 24);
        rsp_arg[3] = (rsp_arg[3] << 8);
    }

    uint8_t csd_struct = mmcsd_parse_csd(&rsp_arg[0]);
    if (csd_struct == 0)
    {
        // SDSC card
        sdsc = 1;
    }
    else if (csd_struct == 1)
    {
        // SDHC card
        sdsc = 0;
    }
    else
    {
        test_result = 'T';
        return test_result;  // CSD structure invalid
    }

#ifdef FPGA
    hwp_sdmmc1->CLKCR = 1 << SD_CLKCR_DIV_Pos; // 48M/2=24M
#else
    hwp_sdmmc1->CLKCR = 2 << SD_CLKCR_DIV_Pos; // 144M/3=48M
#endif
    hwp_sdmmc1->CLKCR |= SD_CLKCR_VOID_FIFO_ERROR;
    hwp_sdmmc1->TOR = 0x00249f00;              // 50ms timeout for 48MHz
    hwp_sdmmc1->CDR = (0 << SD_CDR_ITIMING_Pos);

    // start card transfer
    // CMD7 (SELECT_CARD)
    HAL_Delay_us(20);
    cmd_arg = (uint32_t)rca << 16;
    cmd_result = sd1_send_cmd(7, cmd_arg);
    if (cmd_result == SD_TIMEOUT)
    {
        test_result = '7';
        return test_result;
    }
    else if (cmd_result == SD_CRCERR)
    {
        test_result = '7';
        return test_result;
    }
    sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
    if (rsp_idx != 7)
    {
        test_result = '7';
        return test_result;
    }

    // ACMD6
    HAL_Delay_us(20);
    cmd_arg = wire_mode ? 2 : 0; // select 4-wire mode or 1-wire mode
    cmd_result = sd1_send_acmd(6, cmd_arg, rca);  // CMD55 + ACMD6
    if (cmd_result == SD_TIMEOUT)
    {
        test_result = '6';
        return test_result;
    }
    else if (cmd_result == SD_CRCERR)
    {
        test_result = '6';
        return test_result;
    }

#if 0
    sd1_read(wire_mode, 1); // 4-wire mode, 1 block

    // CMD17 (READ_SINGLE_BLOCK)
    HAL_Delay_us(20);
    cmd_arg = 0; // start data address
    cmd_result = sd1_send_cmd(17, cmd_arg);
    if (cmd_result == SD_TIMEOUT)
    {
        test_result = 'R';
        return test_result; // Read command
    }
    else if (cmd_result == SD_CRCERR)
    {
        test_result = 'R';
        return test_result;
    }
    sd1_get_rsp(&rsp_idx, &rsp_arg[0], &rsp_arg[1], &rsp_arg[2], &rsp_arg[3]);
    if (rsp_idx != 17)
    {
        test_result = 'R';
        return test_result;
    }

    // wait read data
    hwp_sdmmc1->SR  = 0xffffffff;            // clear sdmmc interrupts
    hwp_sdmmc1->IER = SD_IER_DATA_DONE_MASK;
    cmd_result = sd1_wait_read();            // wait sdmmc interrupt
    if (hwp_sdmmc1->SR & SD_SR_DATA_TIMEOUT)
    {
        test_result = 'O';
        return test_result;                   // read timeout
    }
    if (hwp_sdmmc1->SR & SD_SR_DATA_CRC)
    {
        test_result = 'D';
        return test_result;                   // data error
    }
#endif /* if 0 */

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

#ifdef SD_USE_DMA
    // Configure DMA: transfer one block per read (512B = 128 words)
    if (IS_DCACHED_RAM(data))
        SCB_InvalidateDCache_by_Addr(data, len);

    hwp_dmac1->CCR3   = 0;
    hwp_dmac1->CPAR3  = (uint32_t)(uintptr_t)&hwp_sdmmc1->FIFO;
    hwp_dmac1->CM0AR3 = (uint32_t)(uintptr_t)buf;
    hwp_dmac1->CNDTR3 = 512u / 4u;

    // Select DMA request source (clear then set, avoid accumulation)
    hwp_dmac1->CSELR1 = (hwp_dmac1->CSELR1 & ~DMAC_CSELR1_C3S_Msk) |
                        (57u << DMAC_CSELR1_C3S_Pos);  // TODO: replace 57u with the chip's official request ID

    // Peripheral-to-memory, 32-bit, memory increment, non-circular
    hwp_dmac1->CCR3 = DMAC_CCR3_MINC |
                      (0x2u << DMAC_CCR3_MSIZE_Pos) |  // 32-bit
                      (0x2u << DMAC_CCR3_PSIZE_Pos);   // 32-bit
    hwp_dmac1->CCR3 &= ~DMAC_CCR3_CIRC;
    hwp_dmac1->CCR3 |= DMAC_CCR3_EN;
#endif

    // Start data path before read command
    sd1_read(wire_mode, 1);  // 4-wire mode, 1 block

    // CMD17 (READ_SINGLE_BLOCK)
    HAL_Delay_us(20);
    cmd_arg = sdsc ? addr : addr >> 9; // start data address
    cmd_result = sd1_send_cmd(17, cmd_arg);
    if (cmd_result == SD_TIMEOUT)
    {
        test_result = TEST_FAIL;
        return 0;
    }
    else if (cmd_result == SD_CRCERR)
    {
        test_result = TEST_FAIL;
        return 0;
    }
    sd1_get_rsp(&rsp_idx, &rsp_arg1, &rsp_arg2, &rsp_arg3, &rsp_arg4);
    if (rsp_idx != 17)
    {
        test_result = TEST_FAIL;
        return 0;
    }

    // Wait for data (DMA or PIO)
    hwp_sdmmc1->SR  = 0xffffffff;            // clear sdmmc interrupts
    hwp_sdmmc1->IER = SD_IER_DATA_DONE_MASK;
    cmd_result = sd1_wait_read();            // wait sdmmc interrupt
    if (hwp_sdmmc1->SR & SD_SR_DATA_TIMEOUT)
    {
        test_result = TEST_FAIL;
        return 0;
    }
    if (hwp_sdmmc1->SR & SD_SR_DATA_CRC)
    {
        test_result = TEST_FAIL;
        return 0;
    }

    // Wait for DMA drain then disable channel (DMA) or read FIFO (PIO)
#if defined(SD_USE_DMA)
    while (hwp_dmac1->CNDTR3 != 0) { /* wait */ }
    hwp_dmac1->CCR3 &= ~DMAC_CCR3_EN;
#else
    for (i = 0; i < (int)(len / 4); i++)
    {
        *buf = hwp_sdmmc1->FIFO;
        buf++;
    }
#endif /* SD_USE_DMA */

    if (test_result == TEST_FAIL)
    {
        return 0;
    }

    return len;
}

// uint32_t src = 0x20000000;
int sd_read_multi(uint32_t addr, uint8_t *data, uint32_t len)
{
    if (len == 0 || (len % 512u) != 0)
        return 0;

    uint32_t remaining = len;
    uint32_t cur_addr  = addr;
    uint8_t *dst       = data;

#if defined(SD_USE_DMA)
    // Configure the DMA request source once (replace 57u with the official SD RX request ID)
    hwp_dmac1->CSELR1 = (hwp_dmac1->CSELR1 & ~DMAC_CSELR1_C3S_Msk) |
                        (57u << DMAC_CSELR1_C3S_Pos);
#endif

    while (remaining)
    {
        uint32_t blocks = remaining / 512u;
        if (blocks > 255u) blocks = 255u;

#if defined(SD_USE_DMA)
        // DMA continuous multi-block: CMD18 + CMD12
        // if (((uintptr_t)dst & 3u) != 0) return 0;  // 32-bit alignment if required

        uint32_t bytes = blocks * 512u;
        if (IS_DCACHED_RAM(dst))
            SCB_InvalidateDCache_by_Addr(dst, bytes);

        // 1) Disable DMA channel and configure: Periph->Mem, 32-bit, MINC, non-circular
        hwp_dmac1->CCR3  &= ~DMAC_CCR3_EN;
        hwp_dmac1->CPAR3  = (uint32_t)(uintptr_t)&hwp_sdmmc1->FIFO;
        hwp_dmac1->CM0AR3 = (uint32_t)(uintptr_t)dst;
        hwp_dmac1->CNDTR3 = bytes / 4u;
        hwp_dmac1->CCR3   = DMAC_CCR3_MINC |
                            (0x2u << DMAC_CCR3_MSIZE_Pos) |  // MSIZE=32-bit
                            (0x2u << DMAC_CCR3_PSIZE_Pos);   // PSIZE=32-bit
        hwp_dmac1->CCR3  &= ~DMAC_CCR3_CIRC;                 // non-circular

        // 2) Prepare SD data path and enable DMA
        hwp_sdmmc1->SR = 0xFFFFFFFF;                         // clear SD status
        sd1_read(wire_mode, (uint8_t)blocks);                // set data length = blocks*512, enable controller data path
        hwp_dmac1->CCR3 |= DMAC_CCR3_EN;

        // 3) Send CMD18
        // HAL_Delay_us(2);
        uint32_t cmd_arg = sdsc ? cur_addr : (cur_addr >> 9);
        uint32_t rc = sd1_send_cmd(18, cmd_arg);
        if (rc == SD_TIMEOUT || rc == SD_CRCERR)
        {
            hwp_dmac1->CCR3 &= ~DMAC_CCR3_EN;
            return 0;
        }

        uint8_t  rsp_idx;
        uint32_t r1, r2, r3, r4;
        sd1_get_rsp(&rsp_idx, &r1, &r2, &r3, &r4);
        if (rsp_idx != 18)
        {
            hwp_dmac1->CCR3 &= ~DMAC_CCR3_EN;
            return 0;
        }

        // 4) Wait for SD data done and DMA drain
        hwp_sdmmc1->IER = SD_IER_DATA_DONE_MASK;
        (void)sd1_wait_read();
        if (hwp_sdmmc1->SR & (SD_SR_DATA_TIMEOUT | SD_SR_DATA_CRC))
        {
            hwp_dmac1->CCR3 &= ~DMAC_CCR3_EN;
            (void)sd1_send_cmd(12, 0);
            return 0;
        }

        while (hwp_dmac1->CNDTR3 != 0) { /* wait */ }
        hwp_dmac1->CCR3 &= ~DMAC_CCR3_EN;

        // 5) CMD12 stop transmission
        // HAL_Delay_us(2);
        rc = sd1_send_cmd(12, 0);
        if (rc == SD_TIMEOUT || rc == SD_CRCERR)
            return 0;
        sd1_get_rsp(&rsp_idx, &r1, &r2, &r3, &r4);  // R1b
#else
        // PIO continuous multi-block: CMD18 + CMD12
        sd1_read(wire_mode, (uint8_t)blocks);

        HAL_Delay_us(2);
        uint32_t cmd_arg = sdsc ? cur_addr : (cur_addr >> 9);
        uint32_t rc = sd1_send_cmd(18, cmd_arg);
        if (rc == SD_TIMEOUT || rc == SD_CRCERR)
            return 0;

        uint8_t  rsp_idx;
        uint32_t r1, r2, r3, r4;
        sd1_get_rsp(&rsp_idx, &r1, &r2, &r3, &r4);
        if (rsp_idx != 18)
            return 0;

        hwp_sdmmc1->SR  = 0xFFFFFFFF;
        hwp_sdmmc1->IER = SD_IER_DATA_DONE_MASK;
        (void)sd1_wait_read();
        if (hwp_sdmmc1->SR & SD_SR_DATA_TIMEOUT)
            return 0;
        if (hwp_sdmmc1->SR & SD_SR_DATA_CRC)
            return 0;

        uint32_t words = (blocks * 512u) / 4u;
        uint32_t *p32 = (uint32_t *)dst;
        for (uint32_t i = 0; i < words; i++)
        {
            *p32++ = hwp_sdmmc1->FIFO;
        }

        HAL_Delay_us(2);
        rc = sd1_send_cmd(12, 0);
        if (rc == SD_TIMEOUT || rc == SD_CRCERR)
            return 0;
        sd1_get_rsp(&rsp_idx, &r1, &r2, &r3, &r4);  // R1b
#endif

        // advance pointers
        // uint32_t bytes = blocks * 512u;  // already defined in DMA branch; keep same size here
        dst       += blocks * 512u;
        cur_addr  += blocks * 512u;
        remaining -= blocks * 512u;
    }

    return len;
}

static __inline uint32_t GET_BITS(uint32_t *resp, uint32_t  start, uint32_t  size)
{
    const int32_t __size = size;
    const uint32_t __mask = (__size < 32 ? 1u << __size : 0u) - 1u;
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

