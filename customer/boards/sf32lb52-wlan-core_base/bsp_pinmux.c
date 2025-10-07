/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_board.h"

#ifdef BSP_USING_PSRAM1
/* APS 128p*/
static void board_pinmux_psram_func0()
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
static void board_pinmux_psram_func1_2_4(int func)
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
static void board_pinmux_psram_func3()
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

static void board_pinmux_mpi1_none(void)
{
    uint32_t i;

    for (i = 0; i <= 12; i++)
    {
        HAL_PIN_Set_Analog(PAD_SA00 + i, 1);
    }
}
#endif
static void BSP_PIN_Common(void)
{
#ifdef SOC_BF0_HCPU
    // HCPU pins

    uint32_t pid = (hwp_hpsys_cfg->IDR & HPSYS_CFG_IDR_PID_Msk) >> HPSYS_CFG_IDR_PID_Pos;

    pid &= 7;
    if (pid == 0)   // Puya flash
    {
        HAL_PIN_Set(PAD_SA01, MPI1_CS,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA09, MPI1_CLK,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA07, MPI1_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA02, MPI1_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA10, MPI1_DIO3, PIN_NOPULL, 1);

        if (PM_STANDBY_BOOT == SystemPowerOnModeGet())
        {
            /* not support yet, has DIO2 is decided by flash size */
            HAL_ASSERT(0);
        }

        //Should set in bootloader
        //HAL_PIN_Set(PAD_SA03, MPI1_DIO2, PIN_PULLUP, 1);   // 64Mbits only
        //HAL_PIN_Set(PAD_SA00, MPI1_DIO2, PIN_PULLUP, 1);   // 16/32Mbits
    }
    else if (pid == 1)  // GD Flash
    {
        HAL_PIN_Set(PAD_SA04, MPI1_CS,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA09, MPI1_CLK,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA11, MPI1_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA02, MPI1_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA00, MPI1_DIO2, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_SA08, MPI1_DIO3, PIN_PULLUP, 1);

        HAL_PIN_Set_Analog(PAD_SA01, 1);
        HAL_PIN_Set_Analog(PAD_SA03, 1);
        HAL_PIN_Set_Analog(PAD_SA05, 1);
        HAL_PIN_Set_Analog(PAD_SA06, 1);
        HAL_PIN_Set_Analog(PAD_SA07, 1);
        HAL_PIN_Set_Analog(PAD_SA10, 1);
        HAL_PIN_Set_Analog(PAD_SA12, 1);
    }
    else    // psram
    {
#ifdef BSP_USING_PSRAM1
        switch (pid)
        {
        case 5: //BOOT_PSRAM_APS_16P:
            board_pinmux_psram_func3();         // 16Mb APM QSPI PSRAM
            break;
        case 4: //BOOT_PSRAM_APS_32P:
            board_pinmux_psram_func1_2_4(2);    // 32Mb APM LEGACY PSRAM
            break;
        case 6: //BOOT_PSRAM_WINBOND:                // Winbond HYPERBUS PSRAM
            board_pinmux_psram_func1_2_4(4);
            break;
        case 3: // BOOT_PSRAM_APS_64P:
            board_pinmux_psram_func1_2_4(1);    // 64Mb APM XCELLA PSRAM
            break;
        case 2: //BOOT_PSRAM_APS_128P:
            board_pinmux_psram_func0();         // 128Mb APM XCELLA PSRAM
            break;
        default:
            board_pinmux_mpi1_none();
            break;
        }
#endif /* BSP_USING_PSRAM1 */
    }

#ifdef BSP_USING_SDIO
    HAL_PIN_Set(PAD_PA15, SD1_CMD, PIN_PULLUP, 1);
    HAL_Delay_us(20);   // add a delay before clock setting to avoid wrong cmd happen

    HAL_PIN_Set(PAD_PA14, SD1_CLK,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA16, SD1_DIO0, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA17, SD1_DIO1, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA12, SD1_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA13, SD1_DIO3, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA11, GPIO_A11, PIN_PULLUP, 1);
#elif defined(BSP_ENABLE_MPI2)
    // MPI2
    HAL_PIN_Set(PAD_PA16, MPI2_CLK,  PIN_NOPULL,   1);
    HAL_PIN_Set(PAD_PA12, MPI2_CS,   PIN_NOPULL,   1);
    HAL_PIN_Set(PAD_PA15, MPI2_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA13, MPI2_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA14, MPI2_DIO2, PIN_PULLUP,   1);
    HAL_PIN_Set(PAD_PA17, MPI2_DIO3, PIN_PULLUP, 1);
#else
    HAL_PIN_Set(PAD_PA16, GPIO_A16,  PIN_PULLDOWN,   1);
    HAL_PIN_Set(PAD_PA12, GPIO_A12,  PIN_PULLDOWN,   1);
    HAL_PIN_Set(PAD_PA15, GPIO_A15, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA13, GPIO_A13, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA14, GPIO_A14, PIN_PULLDOWN,   1);
    HAL_PIN_Set(PAD_PA17, GPIO_A17, PIN_PULLDOWN, 1);
#endif

#if 1
    // UART1
    HAL_PIN_Set(PAD_PA19, USART1_TXD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA18, USART1_RXD, PIN_PULLUP, 1);
#else
    //SWD
    HAL_PIN_Set(PAD_PA18, SWDIO, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA19, SWCLK, PIN_PULLDOWN, 1);
    HAL_PIN_SetMode(PAD_PA18, 1, PIN_DIGITAL_IO_PULLDOWN);
    HAL_PIN_SetMode(PAD_PA19, 1, PIN_DIGITAL_IO_PULLDOWN);
#endif /* 1 */

    // Key1
    /* Keep default pull-down unchanged. Uart download driver would use this function,
     * if pulldown is disabled, download driver would not work on the board without external pull-down
     */
    // HAL_PIN_Set(PAD_PA34, GPIO_A34, PIN_NOPULL, 1);
    // Key2
    HAL_PIN_Set(PAD_PA33, GPIO_A33, PIN_NOPULL, 1);

    HAL_PIN_Set(PAD_PA21, GPIO_A21, PIN_NOPULL, 1);
#endif

}

void BSP_PIN_Touch(void)
{
    // Touch
    HAL_PIN_Set(PAD_PA09, GPIO_A9,  PIN_NOPULL, 1);    // TP_INT
    HAL_PIN_Set(PAD_PA10, GPIO_A10, PIN_NOPULL, 1);    // TP_RESET
    HAL_PIN_Set(PAD_PA20, I2C1_SCL, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA11, I2C1_SDA, PIN_PULLUP, 1);
}

void BSP_PIN_LCD(void)
{

#ifdef BSP_LCDC_USING_QADSPI
    HAL_PIN_Set(PAD_PA31, GPIO_A31, PIN_NOPULL, 1);   // LCD_RSTB
    HAL_PIN_Set(PAD_PA28, GPIO_A28, PIN_NOPULL, 1);   // LCD_AVDD_EN

    // LCDC1 - QSPI
    HAL_PIN_Set(PAD_PA02, LCDC1_SPI_TE, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA03, LCDC1_SPI_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA04, LCDC1_SPI_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA05, LCDC1_SPI_DIO0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA06, LCDC1_SPI_DIO1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA07, LCDC1_SPI_DIO2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA08, LCDC1_SPI_DIO3, PIN_NOPULL, 1);

    BSP_PIN_Touch();
    // A37, 39-43 GPIOs
#elif defined(BSP_LCDC_USING_DBI)
    HAL_PIN_Set(PAD_PA28, GPTIM1_CH4, PIN_NOPULL, 1);   // LCDC1_BL_PWM_CTRL, LCD backlight PWM

    // LCDC1 - QSPI
    HAL_PIN_Set(PAD_PA02, LCDC1_8080_TE, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA03, LCDC1_8080_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA04, LCDC1_8080_WR, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA05, LCDC1_8080_RD, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA06, LCDC1_8080_DC, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA07, LCDC1_8080_DIO0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA08, LCDC1_8080_DIO1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA37, LCDC1_8080_DIO2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA39, LCDC1_8080_DIO3, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA40, LCDC1_8080_DIO4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA41, LCDC1_8080_DIO5, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA42, LCDC1_8080_DIO6, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA43, LCDC1_8080_DIO7, PIN_NOPULL, 1);

    BSP_PIN_Touch();
    // A37, 39-43 GPIOs
#elif defined(BSP_LCDC_USING_SPI_DCX_1DATA)
    // LCDC1 - SPI
    HAL_PIN_Set(PAD_PA02, LCDC1_SPI_TE, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA03, LCDC1_SPI_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA04, LCDC1_SPI_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA05, LCDC1_SPI_DIO0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA06, LCDC1_SPI_DIO1, PIN_NOPULL, 1);

    // GPIOs A00, A7-A9, A26, A30, A33, A39-A43
#else
    /* disable compile error as LCD may be disabled by some example, such as hal_example */
// #error LCD type not supported in this board.
#endif

}

void BSP_PIN_Init(void)
{
    BSP_PIN_Common();

    BSP_PIN_LCD();

}

