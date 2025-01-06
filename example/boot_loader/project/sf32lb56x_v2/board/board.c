/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-21     zylx         first version
 */

#include "boot_board.h"

void SystemClock_Config(void)
{
}

// Do not use HAL_PIN_Get in bootloader ROM
pin_function HAL_PIN_Idx2Func(int pad, int idx,  int hcpu)
{
    return 0;
}

void board_pinmux_psram(uint8_t       mpi, uint8_t type)
{
    //int mpi = BOOT_MPI1_PSRAM, type = SPI_MODE_OPSRAM;

    // TODO: Package decide mpi and type of PSRAM

    // enable psram power for BGA
    HAL_PIN_Set(PAD_PA72, GPIO_A72, PIN_PULLUP, 1);               // SB_EN/MPI2_EN
    HAL_PIN_Set(PAD_PA74, GPIO_A74, PIN_PULLUP, 1);               // SA_EN/MPI1_EN

    if (mpi & BOOT_MPI1_PSRAM)
    {
        if (type == SPI_MODE_LEGPSRAM)
        {
            HAL_PIN_Set(PAD_SA00, MPI1_DM,   PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA01, MPI1_DIO0, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA02, MPI1_DIO1, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA03, MPI1_DIO2, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA04, MPI1_DIO3, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA05, MPI1_CS, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA06, MPI1_CLKB, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA07, MPI1_CLK,  PIN_NOPULL, 1);
            HAL_PIN_Set(PAD_SA08, MPI1_DIO4, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA09, MPI1_DIO5, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA10, MPI1_DIO6, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA11, MPI1_DIO7, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA12, MPI1_DQS, PIN_PULLDOWN, 1);
        }
        else if (type == SPI_MODE_OPSRAM)
        {

            HAL_PIN_Set(PAD_SA01, MPI1_DIO0, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA02, MPI1_DIO1, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA03, MPI1_DIO2, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA04, MPI1_DIO3, PIN_PULLDOWN, 1);

            HAL_PIN_Set(PAD_SA06, MPI1_CS,   PIN_NOPULL, 1);
            HAL_PIN_Set(PAD_SA07, MPI1_CLK,  PIN_NOPULL, 1);
            HAL_PIN_Set(PAD_SA08, MPI1_DIO4, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA09, MPI1_DIO5, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA10, MPI1_DIO6, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA11, MPI1_DIO7, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA12, MPI1_DQSDM, PIN_PULLDOWN, 1);
        }
        else   // Hyperbus
        {

            HAL_PIN_Set(PAD_SA01, MPI1_DIO0, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA02, MPI1_DIO1, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA03, MPI1_DIO2, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA04, MPI1_DIO3, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA05, MPI1_CS, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA06, MPI1_CLKB, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA07, MPI1_CLK,  PIN_NOPULL, 1);
            HAL_PIN_Set(PAD_SA08, MPI1_DIO4, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA09, MPI1_DIO5, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA10, MPI1_DIO6, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA11, MPI1_DIO7, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA12, MPI1_DQSDM, PIN_PULLDOWN, 1);
        }

    }

    if (mpi & BOOT_MPI2_PSRAM)
    {
        if (type == SPI_MODE_LEGPSRAM)
        {
            HAL_PIN_Set(PAD_SB00, MPI2_DM,   PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB01, MPI2_DIO0, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB02, MPI2_DIO1, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB03, MPI2_DIO2, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB04, MPI2_DIO3, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB05, MPI2_CS,   PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB06, MPI2_CLKB, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB07, MPI2_CLK,  PIN_NOPULL, 1);
            HAL_PIN_Set(PAD_SB08, MPI2_DIO4, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB09, MPI2_DIO5, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB10, MPI2_DIO6, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB11, MPI2_DIO7, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB12, MPI2_DQS,  PIN_PULLDOWN, 1);
        }
        else
        {

            HAL_PIN_Set(PAD_SB01, MPI2_DIO0, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB02, MPI2_DIO1, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB03, MPI2_DIO2, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB04, MPI2_DIO3, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB05, MPI2_DIO4, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB06, MPI2_DIO5, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB07, MPI2_DIO6, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB08, MPI2_DIO7, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SB09, MPI2_DQSDM, PIN_PULLDOWN, 1);

            HAL_PIN_Set(PAD_SB11, MPI2_CLK,  PIN_NOPULL, 1);
            HAL_PIN_Set(PAD_SB12, MPI2_CS,   PIN_NOPULL, 1);
        }
    }
}

void board_pinmux_mpi3(int qfn)
{
    if (qfn)
    {
        HAL_PIN_Set(PAD_PA06, MPI3_CS,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_PA07, MPI3_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA08, MPI3_DIO2, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA09, MPI3_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA10, MPI3_CLK, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_PA11, MPI3_DIO3, PIN_PULLUP, 1);
    }
    else
    {
        // enable mpi3 power for BGA
        HAL_PIN_Set(PAD_PA03, GPIO_A3,  PIN_PULLUP, 1);

        HAL_PIN_Set(PAD_PA06, MPI3_CS,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_PA07, MPI3_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA08, MPI3_DIO2, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA09, MPI3_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA10, MPI3_CLK, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_PA11, MPI3_DIO3, PIN_PULLUP, 1);
    }
}

void board_pinmux_sd1()
{
    HAL_PIN_Set(PAD_PA12, SD1_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA13, SD1_DIO6, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA14, SD1_DIO7, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA15, SD1_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA19, SD1_DIO5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA20, SD1_DIO3, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA21, SD1_DIO4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA22, SD1_DIO0, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA26, SD1_CLK, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA27, SD1_CMD, PIN_PULLUP, 1);

    HAL_PIN_Set(PAD_PA05, GPIO_A5,  PIN_PULLUP, 1);   // SD1_EN
    HAL_PIN_Set(PAD_PA23, GPIO_A23, PIN_PULLUP, 1);    // SD1 Reset

    HAL_PIN_Set(PAD_PB04, TWI_CLK, PIN_PULLUP, 0);    // Connect SF30147C
    HAL_PIN_Set(PAD_PB05, TWI_DIO, PIN_PULLUP, 0);
}


void board_pinmux_uart(int qfn)
{
    if (qfn)
    {
        HAL_PIN_Set(PAD_PA17, USART1_TXD, PIN_NOPULL, 1);   // KEY1/UART1_TXD
        HAL_PIN_Set(PAD_PA18, USART1_RXD, PIN_PULLUP, 1);   // KEY2/UART1_RXD
    }
    else
    {
        HAL_PIN_Set(PAD_PA30, USART1_RXD, PIN_PULLUP, 1);   // UART1_RXD
        HAL_PIN_Set(PAD_PA34, USART1_TXD, PIN_PULLUP, 1);   // UART1_TXD
    }
}

#if 0
int board_boot_from(void)
{
    // TODO: Get boot NAND/NOR, MPI from ptab.h?

    uint32_t pid = 0;
    int r;

    if (pid == 0)
    {
        r = BOOT_FROM_QFN_MPI3_NAND;
    }
    else
    {
        r = BOOT_FROM_BGA_MPI3_NAND;
    }


    return r;
}
#endif

// Use internal LDO to power on flash.
void board_flash_power_on()
{
    // Power on flash
    HAL_PBR0_FORCE1_DISABLE();
    MODIFY_REG(hwp_rtc->PBR0R, RTC_PBR0R_SEL_Msk, MAKE_REG_VAL(1, RTC_PBR0R_SEL_Msk, RTC_PBR0R_SEL_Pos));// cancle PBR0 force 1
    HAL_PBR_ConfigMode(0, 1);
    HAL_PBR_WritePin(0, 0);     // set PBR0 0

    HAL_Delay_us(1500);

    MODIFY_REG(hwp_rtc->PBR0R, RTC_PBR0R_SEL_Msk, MAKE_REG_VAL(0, RTC_PBR0R_SEL_Msk, RTC_PBR0R_SEL_Pos));// recover PBR0 force 1
    HAL_PBR0_FORCE1_ENABLE();
    HAL_Delay_us(5000);
    return;

}

#define WR_REG(a, b)      ( *((uint32_t *)(a)) = (b))
#define RD_REG(a)         (*((uint32_t *)(a)))

#define EUROPA_PMIC_REG_BASE        (0x50070000)
#define EUROPA_PMIC_LDO33_VOUT          (0x20)
#define EUROPA_PMIC_1V8_LVSW100_5       (0x30)

void board_sd1_power_on()
{
    // enable emmc 3v3, LDO33_VOUT
    //WR_REG((EUROPA_PMIC_REG_BASE+EUROPA_PMIC_LDO33_VOUT), (0xd<<3)|1);
    uint32_t data = RD_REG(EUROPA_PMIC_REG_BASE + EUROPA_PMIC_LDO33_VOUT);
    data |= 1;
    WR_REG((EUROPA_PMIC_REG_BASE + EUROPA_PMIC_LDO33_VOUT), data);

    // enable emmc18, LVSW100_5
    data = RD_REG(EUROPA_PMIC_REG_BASE + EUROPA_PMIC_1V8_LVSW100_5);
    data |= (1 << 5);
    WR_REG((EUROPA_PMIC_REG_BASE + EUROPA_PMIC_1V8_LVSW100_5), data);
}


