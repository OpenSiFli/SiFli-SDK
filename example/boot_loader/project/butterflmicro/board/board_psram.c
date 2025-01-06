/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-21     zylx         first version
 */

#include "board.h"
#include "boot_flash.h"
#include "string.h"

enum
{
    BF0_QPSRAM_RD   = 0x03,   // Read
    BF0_QPSRAM_FRD  = 0x0b,   // Fast Read
    BF0_QPSRAM_QRD  = 0xeb,   // Quad Read
    BF0_QPSRAM_WR   = 0x02,   // Write
    BF0_QPSRAM_QWR  = 0x38,   // Quad Write
    BF0_QPSRAM_WRD  = 0x8b,   // Wrapped Read
    BF0_QPSRAM_WWR  = 0x82,   // Wrapped Write
    BF0_QPSRAM_MRR  = 0xb5,   // Mode Register Read
    BF0_QPSRAM_MRW  = 0xb1,   // Mode Register Write
    BF0_QPSRAM_QEN  = 0x35,   // Quad Mode Enable
    BF0_QPSRAM_QDIS = 0xf5,   // Quad Mode Disable
    BF0_QPSRAM_RSTE = 0x66,   // Reset Enable
    BF0_QPSRAM_RST  = 0x99,   // Reset
    BF0_QPSRAM_BLT  = 0xc0,   // Burst Length Toggle
    BF0_QPSRAM_RDID = 0x9f    // Read ID
};

enum
{
    OPSRAM_RD   = 0x00,   // Read
    OPSRAM_WR   = 0x80,   // Write
    OPSRAM_LRD  = 0x20,   // Linear Read
    OPSRAM_LWR  = 0xa0,   // Linear Write
    OPSRAM_MRR  = 0x40,   // Mode Register Read
    OPSRAM_MRW  = 0xc0,   // Mode Register Write
    OPSRAM_RST  = 0xff    // Global Reset
};

/* APS 128p*/
void board_pinmux_psram_func0()
{
    HAL_PIN_Set(PAD_SA01, MPI1_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA02, MPI1_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA03, MPI1_DIO2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA04, MPI1_DIO3, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA05, MPI1_DIO4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA06, MPI1_DIO5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA07, MPI1_DIO6, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA08, MPI1_DIO7, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA09, MPI1_DQSDM, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA10, MPI1_CLK,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SA11, MPI1_CS,   PIN_NOPULL, 1);
    HAL_PIN_Set_Analog(PAD_SA00, 1);
    HAL_PIN_Set_Analog(PAD_SA12, 1);
}

/* APS 1:64p 2:32P, 4:Winbond 32/64/128p*/
void board_pinmux_psram_func1_2_4(int func)
{
    HAL_PIN_Set(PAD_SA01, MPI1_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA02, MPI1_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA03, MPI1_DIO2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA04, MPI1_DIO3, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA08, MPI1_DIO4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA09, MPI1_DIO5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA10, MPI1_DIO6, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA11, MPI1_DIO7, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA07, MPI1_CLK,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SA05, MPI1_CS,   PIN_NOPULL, 1);

#ifdef FPGA
    HAL_PIN_Set(PAD_SA00, MPI1_DM, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA06, MPI1_CLKB, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SA12, MPI1_DQSDM, PIN_PULLDOWN, 1);
#else
    switch (func)
    {
    case 1:             // APS 64P XCELLA
        HAL_PIN_Set(PAD_SA12, MPI1_DQSDM, PIN_PULLDOWN, 1);
        HAL_PIN_Set_Analog(PAD_SA00, 1);
        HAL_PIN_Set_Analog(PAD_SA06, 1);
        break;
    case 2:             // APS 32P LEGACY
        HAL_PIN_Set(PAD_SA00, MPI1_DM, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA12, MPI1_DQS, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA06, MPI1_CLKB, PIN_NOPULL, 1);
        break;
    case 4:             // Winbond 32/64/128p
        //HAL_PIN_Set(PAD_SA06, MPI1_CLKB, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA12, MPI1_DQSDM, PIN_NOPULL, 1);
        HAL_PIN_Set_Analog(PAD_SA00, 1);
        HAL_PIN_Set_Analog(PAD_SA06, 1);
        break;
    }
#endif
}


/* APS 16p*/
void board_pinmux_psram_func3()
{
    HAL_PIN_Set(PAD_SA09, MPI1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SA08, MPI1_CS,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SA05, MPI1_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA07, MPI1_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA06, MPI1_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_SA10, MPI1_DIO3, PIN_PULLUP, 1);

    HAL_PIN_Set_Analog(PAD_SA00, 1);
    HAL_PIN_Set_Analog(PAD_SA01, 1);
    HAL_PIN_Set_Analog(PAD_SA02, 1);
    HAL_PIN_Set_Analog(PAD_SA03, 1);
    HAL_PIN_Set_Analog(PAD_SA04, 1);
    HAL_PIN_Set_Analog(PAD_SA11, 1);
    HAL_PIN_Set_Analog(PAD_SA12, 1);
}


void bootloader_switch_clock(int mpi)
{
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_FLASH_DLL2);
}


static FLASH_HandleTypeDef psram_hdl;

void boot_psram_init(int func)
{
    HAL_StatusTypeDef res;
    FLASH_HandleTypeDef *handle;
    uint32_t sys_clk;
    qspi_configure_t qspi_cfg;
    uint8_t r_lat, w_lat;

    /* init as 0 in case reach here before bss is initialized */
    memset(&psram_hdl, 0, sizeof(psram_hdl));
    memset(&qspi_cfg, 0, sizeof(qspi_cfg));
    qspi_cfg.Instance = hwp_mpi1;
    qspi_cfg.base = MPI1_MEM_BASE;
    qspi_cfg.SpiMode = SPI_MODE_OPSRAM;
    switch (func)
    {
    case BOOT_PSRAM_APS_128P:
        qspi_cfg.msize = 16;
        break;
    case BOOT_PSRAM_APS_64P:
        qspi_cfg.msize = 8;
        break;
    case BOOT_PSRAM_APS_32P:
        qspi_cfg.SpiMode = SPI_MODE_LEGPSRAM;
        qspi_cfg.msize = 4;
        break;
    case BOOT_PSRAM_APS_16P:
        qspi_cfg.SpiMode = SPI_MODE_PSRAM;
        qspi_cfg.msize = 2;
        break;
    case BOOT_PSRAM_WINBOND:
        qspi_cfg.SpiMode = SPI_MODE_HBPSRAM;
        qspi_cfg.msize = 8;                   // Might be 16, depends on PSRAM ID, 4MB not support.
        break;
    default:
        HAL_ASSERT(0);
    }

    handle = &psram_hdl;
    handle->wakeup = 0;
    if (qspi_cfg.SpiMode == SPI_MODE_OPSRAM)  // exla opi psram
    {
        uint32_t fix_lat = 1;

        res = HAL_OPI_PSRAM_Init(handle, &qspi_cfg, BSP_GetFlash1DIV());
        HAL_MPI_MR_WRITE(handle, 8, 3);

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
        HAL_FLASH_SET_AHB_RCMD(handle, OPSRAM_RD);
        HAL_FLASH_CFG_AHB_WCMD(handle, 7, w_lat - 1, 0, 0, 3, 7, 7);
        HAL_FLASH_SET_AHB_WCMD(handle, OPSRAM_WR);

        HAL_MPI_SET_FIXLAT(handle, fix_lat, r_lat, w_lat);

    }
    else if (qspi_cfg.SpiMode == SPI_MODE_LEGPSRAM)  // legacy opi psram
    {
        HAL_LEGACY_PSRAM_Init(handle, &qspi_cfg, BSP_GetFlash1DIV());
        HAL_LEGACY_CFG_READ(handle);
        HAL_LEGACY_CFG_WRITE(handle);
    }
    else if (qspi_cfg.SpiMode == SPI_MODE_HBPSRAM)  // Winbond hyperbus psram
    {
        // TODO initalize Winbond Hyperbus PSRAM
        // Need to read ID to decide real qspi_cfg.msize
        HAL_HYPER_PSRAM_Init(handle, &qspi_cfg, BSP_GetFlash1DIV());
        HAL_HYPER_CFG_READ(handle);
        HAL_HYPER_CFG_WRITE(handle);
    }
    else    // qspi psram
    {
        res = HAL_SPI_PSRAM_Init(handle, &qspi_cfg, BSP_GetFlash1DIV());

        /* enable quadline read */
        HAL_FLASH_CFG_AHB_RCMD(handle, 3, 6, 0, 0, 2, 3, 1);
        res = HAL_FLASH_SET_AHB_RCMD(handle, BF0_QPSRAM_QRD);

        /* enable quadline write */
        HAL_FLASH_CFG_AHB_WCMD(handle, 3, 0, 0, 0, 2, 3, 1);
        res = HAL_FLASH_SET_AHB_WCMD(handle, BF0_QPSRAM_QWR);
    }
}


void board_init_psram()
{
    uint32_t pid = (hwp_hpsys_cfg->IDR & HPSYS_CFG_IDR_PID_Msk) >> HPSYS_CFG_IDR_PID_Pos;

    pid = (pid & 0x7);

    switch (pid)
    {
    case BOOT_PSRAM_APS_16P:
        board_pinmux_psram_func3();         // 16Mb APM QSPI PSRAM
        break;
    case BOOT_PSRAM_APS_32P:
        board_pinmux_psram_func1_2_4(2);    // 32Mb APM LEGACY PSRAM
        break;
    case BOOT_PSRAM_WINBOND:                // Winbond HYPERBUS PSRAM
        board_pinmux_psram_func1_2_4(4);
        break;
    case BOOT_PSRAM_APS_64P:
        board_pinmux_psram_func1_2_4(1);    // 64Mb APM XCELLA PSRAM
        break;
    case BOOT_PSRAM_APS_128P:
        board_pinmux_psram_func0();         // 128Mb APM XCELLA PSRAM
        break;
    default:
        return;
    }

    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO_1V8, true, true);

#ifndef CFG_BOOTROM
    bootloader_switch_clock(1);
#endif
    BSP_SetFlash1DIV(2);    // for QSPI PSRAM need set divider to avoid to high, OPI PSRAM do not care.
    boot_psram_init(pid);
}


