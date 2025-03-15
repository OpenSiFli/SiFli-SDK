/**
  ******************************************************************************
  * @file   bf0_hal_mpi_psram.c
  * @author Sifli software development team
  * @brief   QSPI extension HAL module driver.
  *
  ******************************************************************************
*/
/**
 *
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "bf0_hal.h"
#include <string.h>
#include <stdlib.h>

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup PSRAM PSRAM
  * @brief PSRAM HAL module driver
  * @{
  */

#if defined(HAL_MPI_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

#if defined(SF32LB56X) || defined(SF32LB52X)
    static int HAL_MPI_OPSRAM_CAL_DELAY(FLASH_HandleTypeDef *hflash, uint8_t *sck, uint8_t *dqs);
#endif

__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_PSRAM_Init(FLASH_HandleTypeDef *hflash, qspi_configure_t *cfg, uint16_t clk_div)
{
    HAL_StatusTypeDef res;
    uint16_t cs_min;
    uint16_t cs_max;
    uint16_t cshmin;
    uint16_t trcmin;

    if (hflash == NULL || cfg == NULL)
        return HAL_ERROR;

    HAL_QSPI_Init(hflash, cfg);
    //TODO: delay
    //HAL_FLASH_SET_CLK_rom(hflash, clk_div * 2); // for qspi psram, clock divider not same to OPI
    HAL_FLASH_SET_CLK_rom(hflash, clk_div); // for qspi psram, clock divider not same to OPI

    //HAL_QSPI_SET_CLK_INV(hflash, 1, 0);

    uint32_t freq = HAL_QSPI_GET_CLK(hflash);

    /* 8us */
    //cs_max = ((8UL * freq / 1000000) / 10) * 10;

    if (freq <= 24 * 1000000)   // 24MH
    {
        cs_max = 180; //< 4us
    }
    else if (freq <= 48 * 1000000)         // 48M
    {
        cs_max = 380;         // < 4us
    }
    else if (freq <= 96 * 1000000)    // 96M
    {
        cs_max = 760;        // < 4us
    }
    else if (freq <= 120 * 1000000)    // 120M
    {
        cs_max = 950;        // < 4us
    }
    else //if (freq >= 192000000)    // 144M
    {
        cs_max = 1140;        // < 4us
    }

    if (freq > 60 * 1000000)
        HAL_QSPI_SET_CLK_INV(hflash, 1, 0);
    else
        HAL_QSPI_SET_CLK_INV(hflash, 0, 0);

    cs_min = 0;
    cshmin = 2;
    trcmin = 0;

    res = HAL_FLASH_SET_CS_TIME(hflash, cs_min, cs_max, cshmin, trcmin);
    if (res != HAL_OK)
        return res;
    res = HAL_FLASH_SET_ROW_BOUNDARY(hflash, 6);
    if (res != HAL_OK)
        return res;

    // enable QSPI
    HAL_FLASH_ENABLE_QSPI(hflash, 1);
#if 0
    /* enable quadline read */
    HAL_FLASH_CFG_AHB_RCMD(hflash, 3, 6, 0, 0, 2, 3, 1);
    res = HAL_FLASH_SET_AHB_RCMD(hflash, BF0_QPSRAM_QRD);
    if (res != HAL_OK)
        return res;

    /* enable quadline write */
    HAL_FLASH_CFG_AHB_WCMD(hflash, 3, 0, 0, 0, 2, 3, 1);
    res = HAL_FLASH_SET_AHB_WCMD(hflash, BF0_QPSRAM_QWR);
    if (res != HAL_OK)
        return res;
#endif
    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_OPI_PSRAM_Init(FLASH_HandleTypeDef *hflash, qspi_configure_t *cfg, uint16_t clk_div)
{
    //HAL_StatusTypeDef res;
    uint16_t cs_min;
    uint16_t cs_max;
    uint16_t cshmin;
    uint16_t trcmin;
    //uint32_t sys_clk;
    uint8_t dqs_dly;
    uint8_t sck_dly;

    if (hflash == NULL || cfg == NULL)
        return HAL_ERROR;

    HAL_QSPI_Init(hflash, cfg);

    int cren = __HAL_MPI_GET_CREN(hflash);

#if defined(SF32LB56X) || defined(SF32LB52X)

    HAL_MPI_OPSRAM_CAL_DELAY(hflash, &sck_dly, &dqs_dly);
#endif

    //TODO: delay
    //HAL_FLASH_SET_CLK_rom(hflash, clk_div);
    HAL_FLASH_SET_CLK_rom(hflash, 1);


    cs_min = 6;
#ifdef FPGA // for fpga
    cs_max = 190;
    cs_min = 6;
    cshmin = 0;
    trcmin = 3;
#ifndef  SF32LB56X
    dqs_dly = 14;
    sck_dly = 14;
#endif
    hflash->ecc_en = 5;  //rdcyc
    hflash->buf_mode = 5;  //wdcyc
#else //asic
    uint32_t freq = HAL_QSPI_GET_CLK(hflash);
    freq /= 2;

#if 0 // def SF32LB52X //disable it by default, only open it when debug needed!
    // add calibarate delay check for special case
    if (freq == 24 * 1000000)
    {
        // middle value is 0x3f
        if ((sck_dly > (0x3f + 8)) || (sck_dly < (0x3f - 8)))
            while (1);
    }
    else if (freq == 144 * 1000000)
    {
        // middle value is 0x10
        if ((sck_dly > (0x10 + 5)) || (sck_dly < (0x10 - 5)))
            while (1);
    }
#endif
    if (freq <= 24 * 1000000)
    {
        cs_max = 180; //950;
        cshmin = 0; //3;
        trcmin = 3; //14;
#ifdef  SF32LB58X
        dqs_dly = 15;
        sck_dly = 15;
#endif
        hflash->ecc_en = 3;  //rdcyc
        hflash->buf_mode = 3;  //wdcyc
    }
    else if (freq <= 120 * 1000000)
    {
        cs_max = 950; //950;
        cshmin = 3; //3;
        trcmin = 14; //14;
#ifdef  SF32LB58X
        dqs_dly = 14;
        sck_dly = 14;
#endif
        hflash->ecc_en = 5;  //rdcyc
        hflash->buf_mode = 5;  //wdcyc
    }
    else if (freq <= 144 * 1000000)
    {
        cs_max = 1140; //950;
        cshmin = 5; //3;
        trcmin = 17; //14;
#ifdef  SF32LB58X
        dqs_dly = 10;
        sck_dly = 10;
#endif
        hflash->ecc_en = 6;  //rdcyc
        hflash->buf_mode = 6;  //wdcyc
    }
    else // 168M
    {
        cs_max = 1330; //1140; //950;
        cshmin = 8; //5; //3;
        trcmin = 20; //17; //14;
#ifdef  SF32LB58X
        dqs_dly = 7;
        sck_dly = 7;
#endif
        hflash->ecc_en = 7;  //rdcyc
        hflash->buf_mode = 7;  //wdcyc
    }

#endif


    HAL_FLASH_SET_CS_TIME(hflash, cs_min, cs_max, cshmin, trcmin);

    HAL_FLASH_SET_ROW_BOUNDARY(hflash, 7);

    HAL_MPI_ENABLE_DQS(hflash, 1);

    HAL_MPI_SET_DQS_DELAY(hflash, dqs_dly);  //14
    HAL_MPI_SET_SCK(hflash, sck_dly, 0); //14

    // enable QSPI
    HAL_FLASH_ENABLE_QSPI(hflash, 1);
    // enable OPI
    HAL_FLASH_ENABLE_OPI(hflash, 1);

    if ((cren == 0) && (AON_PMR_STANDBY != HAL_HPAON_GET_POWER_MODE()))
        HAL_PSRAM_RESET(hflash);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LEGACY_PSRAM_Init(FLASH_HandleTypeDef *hflash, qspi_configure_t *cfg, uint16_t clk_div)
{
    //HAL_StatusTypeDef res;
    uint16_t cs_min;
    uint16_t cs_max;
    uint16_t cshmin;
    uint16_t trcmin;
    //uint32_t sys_clk;
    uint8_t mr0, mr4;
    uint8_t dqs_dly;
    uint8_t sck_dly;
    uint8_t fix_lat = 1;

    if (hflash == NULL || cfg == NULL)
        return HAL_ERROR;

    HAL_QSPI_Init(hflash, cfg);

    int cren = __HAL_MPI_GET_CREN(hflash);

#if defined(SF32LB56X) || defined(SF32LB52X)

    HAL_MPI_OPSRAM_CAL_DELAY(hflash, &sck_dly, &dqs_dly);
    //dqs_dly = 0xa;
    //sck_dly = 0xa;
#else
    dqs_dly = 0xa;
    sck_dly = 0xa;
#endif
    //TODO: delay
    //HAL_FLASH_SET_CLK_rom(hflash, clk_div);
    HAL_FLASH_SET_CLK_rom(hflash, 1);

    uint32_t freq = HAL_QSPI_GET_CLK(hflash);
    freq /= 2;

#if 0//def SF32LB52X    // disable it by default, only open it when debug needed!
    HPSYS_DvfsModeTypeDef dvfs = HAL_RCC_HCPU_GetCurrentDvfsMode();
    uint8_t thd = 4;
    uint8_t mid = 0x10;
    // add calibarate delay check for special case
    if (freq == 24 * 1000000)
    {
        mid = 0x3f;
        thd = 8;
        // middle value is 0x3f
        if ((sck_dly > (mid + thd)) || (sck_dly < (mid - thd)))
        {
            hflash->Instance->NONCEA = (1 << 16) | (sck_dly << 8) | dqs_dly;
            while (1);
        }
    }
    else if (freq == 144 * 1000000)
    {
        if (dvfs == HPSYS_DVFS_MODE_S1)
        {
            thd = 4;
            mid = 0xf;
        }
        else if (dvfs == HPSYS_DVFS_MODE_S0)
        {
            thd = 4;
            mid = 0xb;
        }
        // else ? something wrong?

        // middle value is 0x10
        if ((sck_dly > (mid + thd)) || (sck_dly < (mid - thd)))
        {
            hflash->Instance->NONCEA = (sck_dly << 8) | dqs_dly;
            while (1);
        }
    }
#endif

#ifdef FPGA // for fpga
    cs_max = 188;
    cs_min = 6;
    cshmin = 0;
    trcmin = 3;
#else //asic
    cs_min = 6;
    if (freq <= 24 * 1000000)   // 24MH
    {
        cs_max = 180; //< 4us
        cshmin = 0; // 1 cycle of 48M > 15ns
        trcmin = 3; // > 60ns
    }
    else if (freq <= 120000000)         // 120M
    {
        cs_max = 950;         // < 4us
        cshmin = 3;           // 4 cycles of 240M > 15ns
        trcmin = 14;          // > 60ns
    }
    else if (freq <= 144000000)    // 144M
    {
        cs_max = 1140;        // < 4us
        cshmin = 5;           // 6 cycles of 288M > 18ns
        trcmin = 17;          // > 60ns
    }
    else if (freq <= 168000000)    // 168M
    {
        cs_max = 1330;        // < 4us
        cshmin = 8;           // 9 cycles of 336M > 24ns
        trcmin = 20;          // > 60ns
    }
    else //if (freq >= 192000000)    // 192M
    {
        cs_max = 1520;        // < 4us
        cshmin = 9;           // 10 cycles of 384M > 24ns
        trcmin = 23;          // > 60ns
    }
#endif

    HAL_FLASH_SET_CS_TIME(hflash, cs_min, cs_max, cshmin, trcmin);

    HAL_FLASH_SET_ROW_BOUNDARY(hflash, 7);

    HAL_MPI_ENABLE_DQS(hflash, 1);

    HAL_MPI_SET_DQS_DELAY(hflash, dqs_dly);
    HAL_MPI_SET_SCK(hflash, sck_dly, 0);

    HAL_FLASH_SET_LEGACY(hflash, 1);
    // enable QSPI
    HAL_FLASH_ENABLE_QSPI(hflash, 1);
    // enable OPI
    HAL_FLASH_ENABLE_OPI(hflash, 1);

    // do not wake up from sleep and dp not enabled before, reset for first initial
    if ((cren == 0) && (hflash->wakeup == 0))
        HAL_PSRAM_RESET(hflash);

#ifdef FPGA
    fix_lat = 1;
    uint8_t drv_str = 3;
    uint8_t rd_lat = 4; // < 66m
    uint8_t wr_lat = 0; // < 133m
    mr0 = (fix_lat << 5) | (rd_lat << 2) | drv_str;
    mr4 = (wr_lat << 7);
    hflash->ecc_en = rd_lat;  //rdcyc
    hflash->buf_mode = wr_lat;  //wdcyc
#else
    fix_lat = 1;
    uint8_t drv_str = 3;
    uint8_t rf = 1;
    if (freq <= 120000000)  // 120M
    {
        mr0 = (fix_lat << 5) | (4 << 2) | drv_str;
        mr4 = (0 << 7) | (rf << 3);
        hflash->ecc_en = 4;  //rdcyc
        hflash->buf_mode = 0;  //wdcyc
    }
    else if (freq <= 144000000) // 144M
    {
        mr0 = (fix_lat << 5) | (5 << 2) | drv_str;
        mr4 = (0 << 7) | (rf << 3);
        hflash->ecc_en = 5;  //rdcyc
        hflash->buf_mode = 0;  //wdcyc
    }
    else    // 168M , 192M
    {
        mr0 = (fix_lat << 5) | (6 << 2) | drv_str;
        mr4 = (1 << 7) | (rf << 3);
        hflash->ecc_en = 6;  //rdcyc
        hflash->buf_mode = 2;  //wdcyc
    }
#endif
    HAL_MPI_EN_FIXLAT(hflash, fix_lat);

    // for some case like freq changed, mr should be set even not reset chip
    //if ((cren == 0) && (hflash->wakeup == 0))
    {
        HAL_LEGACY_MR_WRITE(hflash, 0, mr0);
        HAL_LEGACY_MR_WRITE(hflash, 4, mr4);
    }

    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_HYPER_PSRAM_Init(FLASH_HandleTypeDef *hflash, qspi_configure_t *cfg, uint16_t clk_div)
{
    uint16_t mr0;

    HAL_OPI_PSRAM_Init(hflash, cfg, 1);

    uint32_t freq = HAL_QSPI_GET_CLK(hflash);
    freq /= 2;

    // CR0 with 2 byte, bytes should swap for read/write
    if (freq <= 85 * 1000000)
    {
        mr0 = (14 << 12) | 0x078f;
    }
    else if (freq <= 104 * 1000000)
    {
        mr0 = (15 << 12) | 0x078f;
    }
    else if (freq <= 120 * 1000000)
    {
        mr0 = (0 << 12) | 0x078f;
    }
    else if (freq <= 144 * 1000000)
    {
        mr0 = (1 << 12) | 0x078f;
    }
    else // 168M
    {
        mr0 = (2 << 12) | 0x078f;
    }

    HAL_FLASH_ENABLE_HYPER(hflash, 1);

    HAL_HYPER_PSRAM_WriteCR(hflash, 0, mr0);

    HAL_MPI_EN_FIXLAT(hflash, 1);

    return HAL_OK;
}

__HAL_ROM_USED int HAL_MPI_MR_READ(FLASH_HandleTypeDef *hflash, uint8_t addr)
{
    if (hflash == NULL)
        return 0;

    uint8_t rdcyc = hflash->ecc_en;

    HAL_FLASH_MANUAL_CMD(hflash, 0, 7, rdcyc - 1, 0, 0, 3, 7, 7);

    HAL_FLASH_WRITE_DLEN(hflash, 2);

    HAL_FLASH_SET_CMD(hflash, HAL_OPSRAM_MRREAD, addr);

    return HAL_FLASH_READ32(hflash) & 0xff;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_MPI_MR_WRITE(FLASH_HandleTypeDef *hflash, uint32_t addr, uint8_t value)
{
    if (hflash == NULL)
        return HAL_ERROR;

    HAL_FLASH_MANUAL_CMD(hflash, 1, 7, 0, 0, 0, 3, 7, 7);

    HAL_FLASH_WRITE_DLEN(hflash, 2);
    HAL_FLASH_WRITE_WORD(hflash, (uint32_t)value);

    HAL_FLASH_SET_CMD(hflash, HAL_OPSRAM_MRWRITE, addr);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_MPI_SET_FIXLAT(FLASH_HandleTypeDef *hflash, uint32_t fix, uint8_t r_lat, uint8_t w_lat)
{
    uint8_t val, val4;
    if (hflash == NULL)
        return HAL_ERROR;

    uint8_t rlat_arr[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    uint8_t wlat_arr[8] = {0, 0, 0, 0, 4, 2, 6, 1};

    HAL_MPI_EN_FIXLAT(hflash, fix);

    // MR0[7:6] fix 0, [5] Latency type, [4:2] Read latency code, [1:0] drive str
    // MR4[7:5] Write latency code, [4:3] RF rate, [2:0] PASR
    if (fix)
    {
        val = (1 << 5) | (rlat_arr[r_lat / 2] << 2) | 1;
        val4 = ((wlat_arr[w_lat]) << 5) | (0 << 3) | 0;
    }
    else
    {
        val = (0 << 5) | (rlat_arr[r_lat] << 2) | 1;
        val4 = ((wlat_arr[w_lat]) << 5) | (0 << 3) | 0;
    }

    HAL_MPI_MR_WRITE(hflash, 0, val);
    HAL_MPI_MODIFY_RCMD_DELAY(hflash, r_lat - 1); // NOTE  , as digital suggest, 10 to 9,

    HAL_MPI_MR_WRITE(hflash, 4, val4);
    HAL_MPI_MODIFY_WCMD_DELAY(hflash, w_lat - 1); // NOTE  , as digital suggest, 10 to 9,
    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_MPI_SET_REFRESH(FLASH_HandleTypeDef *hflash, uint32_t ref_value)
{
    if (hflash == NULL)
        return HAL_ERROR;

    int val = HAL_MPI_MR_READ(hflash, 4);
    val &= ~(3 << 3);
    val |= ((ref_value & 3) << 3);  // slow refresh
    HAL_MPI_MR_WRITE(hflash, 4, val);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_MPI_PSRAM_SLEEP(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return HAL_ERROR;

    //int mr4;

    int val = HAL_MPI_MR_READ(hflash, 4);
    val |= (1 << 3);    // slow refresh
    HAL_MPI_MR_WRITE(hflash, 4, val);

    HAL_MPI_MR_WRITE(hflash, 6, 0xf0);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_MPI_PSRAM_WAKEUP(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return HAL_ERROR;

    //int mr4;

    int val = HAL_MPI_MR_READ(hflash, 4);
    val &= ~(1 << 3);
    HAL_MPI_MR_WRITE(hflash, 4, val);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_MPI_PSRAM_DPD(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return HAL_ERROR;

    HAL_MPI_MR_WRITE(hflash, 6, 0xC0);

    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_MPI_PSRAM_SET_PASR(FLASH_HandleTypeDef *hflash, uint8_t top, uint8_t deno)
{
    if (hflash == NULL)
        return HAL_ERROR;

    int mr4, div;

    mr4 = HAL_MPI_MR_READ(hflash, 4);
    mr4 &= ~7;
    div = 0;
    if (deno > 1)
    {
        while (deno > 0)
        {
            div++;
            deno = deno >> 1;
            if (div >= 4)
                break;
        }
        if (top)
            mr4 |= (1 << 2);
        mr4 |= div;
    }

    HAL_MPI_MR_WRITE(hflash, 4, mr4 & 0xff);

    return HAL_OK;
}

__HAL_ROM_USED int HAL_LEGACY_MR_READ(FLASH_HandleTypeDef *hflash, uint8_t addr)
{
    uint32_t adsize = 2; //legacy ? 2 : 3;
    uint32_t dcyc = 2 * hflash->ecc_en; //legacy ? rdcyc : rdcyc-1;
    if (hflash == NULL)
        return -1;

    HAL_FLASH_MANUAL_CMD(hflash, 0, 7, dcyc, 0, 0, adsize, 7, 7);

    HAL_FLASH_WRITE_DLEN(hflash, 2);

    HAL_FLASH_SET_CMD(hflash, HAL_OPSRAM_MRREAD, addr);

    return HAL_FLASH_READ32(hflash) & 0xff;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LEGACY_MR_WRITE(FLASH_HandleTypeDef *hflash, uint32_t addr, uint8_t value)
{
    uint32_t adsize = 2; //legacy ? 2 : 3;

    if (hflash == NULL)
        return HAL_ERROR;

    HAL_FLASH_MANUAL_CMD(hflash, 1, 7, 0, 0, 0, adsize, 7, 7);

    HAL_FLASH_WRITE_DLEN(hflash, 4);
    HAL_FLASH_WRITE_WORD(hflash, (uint32_t)value);

    HAL_FLASH_SET_CMD(hflash, HAL_OPSRAM_MRWRITE, addr);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LEGACY_CFG_READ(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return HAL_ERROR;

    uint32_t adsize = 2; //legacy ? 2 : 3;
    uint32_t dcyc = 2 * hflash->ecc_en; //legacy ? 2*rdcyc : 2*rdcyc-1;

    HAL_FLASH_CFG_AHB_RCMD(hflash, 7, dcyc, 0, 0, adsize, 7, 7);
    HAL_FLASH_SET_AHB_RCMD(hflash, HAL_OPSRAM_READ);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LEGACY_CFG_WRITE(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return HAL_ERROR;

    uint32_t adsize = 2; //legacy ? 2 : 3;
    uint32_t dcyc = hflash->buf_mode; //legacy ? wdcyc : wdcyc-1;
    HAL_FLASH_CFG_AHB_WCMD(hflash, 7, dcyc, 0, 0, adsize, 7, 7);
    HAL_FLASH_SET_AHB_WCMD(hflash, HAL_OPSRAM_WRITE);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_PSRAM_RESET(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return HAL_ERROR;

    uint32_t adsize = 2; //legacy ? 2 : 3;
    uint32_t absize = 3; ////legacy ? 3 : 1;
    if (hflash->isNand == SPI_MODE_LEGPSRAM)
    {
        adsize = 2;
        absize = 3;
    }
    else if ((hflash->isNand == SPI_MODE_OPSRAM) || (hflash->isNand == SPI_MODE_HPSRAM))
    {
        adsize = 3;
        absize = 1;
    }
    else    // other mode do not need reset
        return HAL_OK;

    HAL_FLASH_MANUAL_CMD(hflash, 1, 0, 0, absize, 7, adsize, 7, 7);
    HAL_FLASH_SET_CMD(hflash, HAL_OPSRAM_RESET, 0);

    if (hflash->isNand == SPI_MODE_LEGPSRAM) // legacy need reset twice
    {
        HAL_FLASH_MANUAL_CMD(hflash, 1, 0, 0, absize, 7, adsize, 7, 7);
        HAL_FLASH_SET_CMD(hflash, HAL_OPSRAM_RESET, 0);
    }

    HAL_Delay_us(0);
    HAL_Delay_us(3);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LEGACY_SET_REFRESH(FLASH_HandleTypeDef *hflash, uint32_t ref_value)
{
    if (hflash == NULL)
        return HAL_ERROR;

    int mr4;

    mr4 = HAL_LEGACY_MR_READ(hflash, 4);
    mr4 &= ~(1 << 3);
    mr4 |= (ref_value & 1) << 3;

    HAL_LEGACY_MR_WRITE(hflash, 4, mr4 & 0xff);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LEGACY_PSRAM_SLEEP(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return HAL_ERROR;

    HAL_LEGACY_MR_WRITE(hflash, 6, 0xf0);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LEGACY_PSRAM_WAKEUP(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return HAL_ERROR;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_LEGACY_PSRAM_SET_PASR(FLASH_HandleTypeDef *hflash, uint8_t top, uint8_t deno)
{
    if (hflash == NULL)
        return HAL_ERROR;

    int mr4, div;

    mr4 = HAL_LEGACY_MR_READ(hflash, 4);
    mr4 &= ~7;
    div = 0;
    if (deno > 1)
    {
        while (deno > 0)
        {
            div++;
            deno = deno >> 1;
            if (div >= 4)
                break;
        }
        if (top)
            mr4 |= (1 << 2);
        mr4 |= div;
    }

    HAL_LEGACY_MR_WRITE(hflash, 4, mr4 & 0xff);

    return HAL_OK;
}

__HAL_ROM_USED int HAL_QSPI_GET_PSRAM_ID(FLASH_HandleTypeDef *hflash)
{
    uint32_t data;
    //uint8_t mfid, kgd, density;

    //HAL_FLASH_CLEAR_FIFO(hflash, HAL_FLASH_CLR_RX_FIFO);

    // configure CCR
    // read mode, single data line, 24 bit address, single addr line
    HAL_FLASH_MANUAL_CMD(hflash, 0, 1, 0, 0, 0, 2, 1, 1);

    // configure data length : MF ID, KGD, EID(48bit), total 8 byte?
    // if only check MF ID and KGD, set to 2 to get 2 byte
    // set 3 byte to get EID[47:45]
    HAL_FLASH_WRITE_DLEN(hflash, 3);

    // send command, read id command
    HAL_FLASH_SET_CMD(hflash, 0x9f, 0);

    data = HAL_FLASH_READ32(hflash);

    // use 8 bit check to avoid endian changed
    // get MFID, KGD, density(EID[47:45])
    //mfid = (uint8_t)(data&0xff); // APPSRAM_CHECK_ID & 0xff; // 0d
    //kgd = (uint8_t)((data>>8)&0xff); //(APPSRAM_CHECK_ID >> 8) & 0xff ;  // 5d
    //density = (uint8_t)((data>>21)&0x03); // high 3 bit ?

    return data & 0xffffff;
}

__HAL_ROM_USED uint16_t HAL_HYPER_PSRAM_ReadID(FLASH_HandleTypeDef *hflash, uint8_t idx)
{
    if (hflash == NULL)
        return 0;

    HAL_FLASH_MANUAL_CMD(hflash, 0, 7, (hflash->ecc_en - 1), 0, 7, 3, 7, 7);
    HAL_FLASH_WRITE_ABYTE(hflash, idx);
    HAL_FLASH_WRITE_DLEN(hflash, 2);

    HAL_FLASH_SET_CMD(hflash, 0xc0, 0);

    return (uint16_t)(HAL_FLASH_READ32(hflash) & 0xffff);
}

__HAL_ROM_USED uint16_t HAL_HYPER_PSRAM_ReadCR(FLASH_HandleTypeDef *hflash, uint8_t idx)
{
    if (hflash == NULL)
        return 0;

    HAL_FLASH_MANUAL_CMD(hflash, 0, 7, (hflash->ecc_en - 1), 0, 7, 3, 7, 7);
    HAL_FLASH_WRITE_ABYTE(hflash, idx);
    HAL_FLASH_WRITE_DLEN(hflash, 2);

    HAL_FLASH_SET_CMD(hflash, 0xE0, 0X10000);

    return (uint16_t)(HAL_FLASH_READ32(hflash) & 0xffff);
}

__HAL_ROM_USED void HAL_HYPER_PSRAM_WriteCR(FLASH_HandleTypeDef *hflash, uint8_t idx, uint16_t value)
{
    if (hflash == NULL)
        return;

    HAL_FLASH_MANUAL_CMD(hflash, 1, 7, 0, 0, 7, 3, 7, 7);
    HAL_FLASH_WRITE_ABYTE(hflash, idx);
    HAL_FLASH_WRITE_DLEN(hflash, 2);
    HAL_FLASH_WRITE_WORD(hflash, (uint32_t)value);
    HAL_FLASH_SET_CMD(hflash, 0x60, 0X10000);
}

__HAL_ROM_USED void HAL_HYPER_CFG_READ(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return;

    HAL_FLASH_CFG_AHB_RCMD(hflash, 7, hflash->ecc_en - 1, 0, 7, 3, 7, 7);
    //HAL_FLASH_SET_AHB_RCMD(hflash, HAL_OPSRAM_READ);
}

__HAL_ROM_USED void HAL_HYPER_CFG_WRITE(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return;

    HAL_FLASH_CFG_AHB_WCMD(hflash, 7, hflash->buf_mode - 1, 0, 7, 3, 7, 7);
    //HAL_FLASH_SET_AHB_WCMD(hflash, HAL_OPSRAM_WRITE);
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_HYPER_PSRAM_SLEEP(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return HAL_ERROR;

    int cr1;

    cr1 = HAL_HYPER_PSRAM_ReadCR(hflash, 1) ;
    cr1 = cr1 | ((1 << 5) << 8); // for cr, 2 bytes need swap, high to low, low to high

    HAL_HYPER_PSRAM_WriteCR(hflash, 1, cr1);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_HYPER_PSRAM_WAKEUP(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return HAL_ERROR;

    int cr1;

    cr1 = HAL_HYPER_PSRAM_ReadCR(hflash, 1) ;
    cr1 = cr1 & (~((1 << 5) << 8)); // for cr, 2 bytes need swap, high to low, low to high

    HAL_HYPER_PSRAM_WriteCR(hflash, 1, cr1);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_HYPER_PSRAM_DPD(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return HAL_ERROR;

    int cr0;

    cr0 = HAL_HYPER_PSRAM_ReadCR(hflash, 0) ;
    cr0 = cr0 & (~(1 << 7));  // for cr0, BIT 15 for DPD, 1 normal, 0 DPD

    HAL_HYPER_PSRAM_WriteCR(hflash, 0, cr0);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_HYPER_PSRAM_RDPD(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return HAL_ERROR;

    int cr0;

    cr0 = HAL_HYPER_PSRAM_ReadCR(hflash, 0) ;
    cr0 = cr0 | (1 << 7);  // for cr0, BIT 15 for DPD, 1 normal, 0 DPD

    HAL_HYPER_PSRAM_WriteCR(hflash, 1, cr0);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_HYPER_PSRAM_SET_PASR(FLASH_HandleTypeDef *hflash, uint8_t top, uint8_t deno)
{
    if (hflash == NULL)
        return HAL_ERROR;

    int cr1, div;

    cr1 = HAL_HYPER_PSRAM_ReadCR(hflash, 1);
    cr1 &= ~(7 << 2);

    div = 0;
    if (deno > 1)
    {
        while (deno > 0)
        {
            div++;
            deno = deno >> 1;
            if (div >= 4)
                break;
        }
        if (top)
            cr1 |= ((1 << 2) << 2);
        cr1 |= (div << 2);
    }

    HAL_HYPER_PSRAM_WriteCR(hflash, 1, cr1);

    return HAL_OK;
}

// =======================================================================

HAL_StatusTypeDef HAL_MPI_PSRAM_Init(FLASH_HandleTypeDef *handle, qspi_configure_t *qspi_cfg, uint16_t div)
{
    HAL_StatusTypeDef res = HAL_OK;
    uint32_t sys_clk;
    uint8_t r_lat, w_lat=0;

    handle->isNand = (uint8_t)qspi_cfg->SpiMode;
    if (qspi_cfg->SpiMode == SPI_MODE_OPSRAM)  // exla opi psram
    {
        uint32_t fix_lat = 1;

        res = HAL_OPI_PSRAM_Init(handle, qspi_cfg, div);
        HAL_MPI_MR_WRITE(handle, 8, 3);

        //HAL_MPI_SET_REFRESH(handle, 1); // set to low refresh by default

        // TODO: modify read/write delay to meed psram frequency
        sys_clk = HAL_QSPI_GET_CLK(handle);
        sys_clk /= 2;
        if (sys_clk <= 66 * 1000000)
            w_lat = 3;
        else if (sys_clk <= 109 * 1000000)
            w_lat = 4;
        else if (sys_clk <= 133 * 1000000)
            w_lat = 5;
        else if (sys_clk <= 166 * 1000000)
            w_lat = 6;
        else if (sys_clk <= 200 * 1000000)
            w_lat = 7;


        if (fix_lat)
            r_lat = w_lat * 2; //10;
        else
            r_lat = w_lat; // = 6; //5;

        /* configure AHB command */
        HAL_FLASH_CFG_AHB_RCMD(handle, 7, r_lat - 1, 0, 0, 3, 7, 7);
        HAL_FLASH_SET_AHB_RCMD(handle, HAL_OPSRAM_READ);
        HAL_FLASH_CFG_AHB_WCMD(handle, 7, w_lat - 1, 0, 0, 3, 7, 7);
        HAL_FLASH_SET_AHB_WCMD(handle, HAL_OPSRAM_WRITE);

        HAL_MPI_SET_FIXLAT(handle, fix_lat, r_lat, w_lat);
    }
    else if (qspi_cfg->SpiMode == SPI_MODE_LEGPSRAM)  // legacy opi psram
    {
        HAL_LEGACY_PSRAM_Init(handle, qspi_cfg, div);
        HAL_LEGACY_CFG_READ(handle);
        HAL_LEGACY_CFG_WRITE(handle);
    }
    else if (qspi_cfg->SpiMode == SPI_MODE_HPSRAM)  // hpi psram
    {
        uint32_t fix_lat = 1;
        res = HAL_OPI_PSRAM_Init(handle, qspi_cfg, div);

        //HAL_MPI_MR_WRITE(handle, 8, 3);
        HAL_MPI_MR_WRITE(handle, 8, 0x43);
        HAL_FLASH_SET_X16_MODE(handle, 1);
        //HAL_MPI_SET_REFRESH(handle, 1); // set to low refresh by default, 0.5 may cause more

        sys_clk = HAL_QSPI_GET_CLK(handle);
        sys_clk /= 2;
        if (sys_clk <= 66 * 1000000)
            w_lat = 3;
        else if (sys_clk <= 109 * 1000000)
            w_lat = 4;
        else if (sys_clk <= 133 * 1000000)
            w_lat = 5;
        else if (sys_clk <= 166 * 1000000)
            w_lat = 6;
        else if (sys_clk <= 200 * 1000000)
            w_lat = 7;

        if (fix_lat)
            r_lat = w_lat * 2; //10;
        else
            r_lat = w_lat; // = 6; //5;

        //r_lat = 6;
        //w_lat = 3;

        /* configure AHB command */
        HAL_FLASH_CFG_AHB_RCMD(handle, 7, r_lat - 1, 0, 0, 3, 7, 7);
        HAL_FLASH_SET_AHB_RCMD(handle, HAL_OPSRAM_READ);
        HAL_FLASH_CFG_AHB_WCMD(handle, 7, w_lat - 1, 0, 0, 3, 7, 7);
        HAL_FLASH_SET_AHB_WCMD(handle, HAL_OPSRAM_WRITE);

        HAL_MPI_SET_FIXLAT(handle, fix_lat, r_lat, w_lat);
    }
    else if (qspi_cfg->SpiMode == SPI_MODE_HBPSRAM)  // hyper bus psram
    {
        HAL_HYPER_PSRAM_Init(handle, qspi_cfg, div);
        HAL_HYPER_CFG_READ(handle);
        HAL_HYPER_CFG_WRITE(handle);
    }
    else    // qspi psram
    {
        res = HAL_SPI_PSRAM_Init(handle, qspi_cfg, div);

        /* enable quadline read */
        HAL_FLASH_CFG_AHB_RCMD(handle, 3, 6, 0, 0, 2, 3, 1);
        res = HAL_FLASH_SET_AHB_RCMD(handle, HAL_QPSRAM_QRD);

        /* enable quadline write */
        HAL_FLASH_CFG_AHB_WCMD(handle, 3, 0, 0, 0, 2, 3, 1);
        res = HAL_FLASH_SET_AHB_WCMD(handle, HAL_QPSRAM_QWR);
    }
#if defined(SF32LB56X) || defined(SF32LB52X)
    {
        uint16_t timeout = 0xffff;
        HAL_FLASH_SET_WDT(handle, timeout);
    }
#endif

    return res;
}

HAL_StatusTypeDef HAL_MPI_PSRAM_ENT_LOWP(FLASH_HandleTypeDef *hflash, uint8_t psram_type)
{
    if (psram_type == SPI_MODE_LEGPSRAM)  // legacy
    {
        HAL_LEGACY_PSRAM_SLEEP(hflash);
    }
    else if (psram_type == SPI_MODE_HBPSRAM)  // hyper bus
    {
        HAL_HYPER_PSRAM_SLEEP(hflash);
    }
    else if ((psram_type == SPI_MODE_OPSRAM) || (psram_type == SPI_MODE_HPSRAM))    // opi/hpi
    {
        HAL_MPI_PSRAM_SLEEP(hflash);
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_MPI_EXIT_LOWP(FLASH_HandleTypeDef *hflash, uint8_t psram_type)
{
    if (hflash == NULL || hflash->Instance == NULL)
        return HAL_ERROR;

    if (psram_type == SPI_MODE_LEGPSRAM)  // legacy
    {
        // enable QSPI
        HAL_FLASH_ENABLE_QSPI(hflash, 1);
        // enable OPI
        HAL_FLASH_ENABLE_OPI(hflash, 1);

        HAL_FLASH_MANUAL_CMD(hflash, 1, 7, 7, 0, 0, 2, 7, 7);

        HAL_FLASH_WRITE_DLEN(hflash, 4);
        HAL_FLASH_WRITE_WORD(hflash, 0);

        HAL_FLASH_SET_CMD(hflash, 0, 0);

        // add delay to meet psram request
        HAL_Delay_us(100);

        // recover refresh rate
        //HAL_LEGACY_MR_WRITE(hflash, 4, 0);
    }
    else if (psram_type == SPI_MODE_HBPSRAM)  // hyper bus
    {
        // enable QSPI
        HAL_FLASH_ENABLE_QSPI(hflash, 1);
        // enable OPI
        HAL_FLASH_ENABLE_OPI(hflash, 1);

        HAL_FLASH_MANUAL_CMD(hflash, 1, 7, 7, 0, 0, 3, 7, 7);

        // for half sleep, cs pulse 60 ~ 3000 ns, for dpd, cs pulse need 200 ~ 3000 ns
        HAL_FLASH_WRITE_DLEN(hflash, 4);    // 32
        HAL_FLASH_WRITE_WORD(hflash, 0);

        HAL_FLASH_SET_CMD(hflash, 0, 0);

        // for hybrid sleep, wait at least 100 us, for dpd, wait at least 150 us
        HAL_Delay_us(150);
    }
    else if ((psram_type == SPI_MODE_OPSRAM) || (psram_type == SPI_MODE_HPSRAM))    // opi/hpi
    {
        // enable QSPI
        HAL_FLASH_ENABLE_QSPI(hflash, 1);
        // enable OPI
        HAL_FLASH_ENABLE_OPI(hflash, 1);

        HAL_FLASH_MANUAL_CMD(hflash, 1, 7, 7, 0, 0, 3, 7, 7);

        HAL_FLASH_WRITE_DLEN(hflash, 4);
        HAL_FLASH_WRITE_WORD(hflash, 0);

        HAL_FLASH_SET_CMD(hflash, 0, 0);

        // add delay to meet psram request
        // tXHS 150, tXPHS 60, tXDPD 150, tXPDPD 60
        HAL_Delay_us(150);
    }
    // else qspi , do nothing

    return HAL_OK;
}

#if defined(SF32LB56X) || defined(SF32LB52X)

static void HAL_Delay_us_psram(__IO uint32_t us)
{
    static uint32_t sysclk_m2;

    if (us == 0 || sysclk_m2 == 0)
    {
        sysclk_m2 = HAL_RCC_GetHCLKFreq(CORE_ID_DEFAULT) / 1000000;
        if (us == 0)
            return;
    }

    if (sysclk_m2 > 48)   // DLL has extra cost, Reason to be determined.
        us -= 1;
    if (us > 0)          // Extra 1 us for cost of function
    {
        volatile uint32_t i = sysclk_m2 * (us - 1) / 5;
        while (i-- > 0); //need 5 instructions at ARMCC
    }
}

static int HAL_MPI_OPSRAM_CAL_DELAY(FLASH_HandleTypeDef *hflash, uint8_t *sck, uint8_t *dqs)
{
    uint32_t delay;

    if (hflash == NULL)
        return HAL_ERROR;

    hflash->Instance->PSCLR = 2;
    hflash->Instance->MISCR &= ~MPI_MISCR_SCKINV_Msk;
    hflash->Instance->CALCR |= MPI_CALCR_EN;
    //HAL_Delay_us(0);
    //HAL_Delay_us(20);  // add delay to avoid CDC during cal_done polling
    HAL_Delay_us_psram(0);
    HAL_Delay_us_psram(20);
    while (!(hflash->Instance->CALCR & MPI_CALCR_DONE_Msk));
    delay = hflash->Instance->CALCR & MPI_CALCR_DELAY_Msk;
    hflash->Instance->CALCR &= ~MPI_CALCR_EN;

#ifdef SF32LB56X
    *sck = (uint8_t)(delay - 3);
    *dqs = (uint8_t)(delay - 1);
#else    //SF32LB52X
    *sck = (uint8_t)(delay - 1);
    *dqs = (uint8_t)(delay - 4);
#endif
    hflash->Instance->PSCLR = 1;

    return 0;
}

int HAL_MPI_OPSRAM_AUTO_CAL(FLASH_HandleTypeDef *hflash, uint8_t *sck, uint8_t *dqs)
{
    int res;

    res = HAL_MPI_OPSRAM_CAL_DELAY(hflash, sck, dqs);
    if (res != 0)
        return res;

    HAL_MPI_SET_DQS_DELAY(hflash, *dqs);
    HAL_MPI_SET_SCK(hflash, *sck, 0);

    HAL_Delay_us_psram(50);
    return 0;
}

#endif  // defined(SF32LB56X) || defined(SF32LB52X)

#endif  // HAL_MPI_MODULE_ENABLED

/// @} PSRAM

/// @} BF0_HAL_Driver


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
