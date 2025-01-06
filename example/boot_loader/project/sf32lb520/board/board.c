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

void SystemClock_Config(void)
{
}

// Do not use HAL_PIN_Get in bootloader ROM
pin_function HAL_PIN_Idx2Func(int pad, int idx,  int hcpu)
{
    return 0;
}

void board_pinmux_mpi1_puya_base()
{
    HAL_PIN_Set(PAD_SA01, MPI1_CS,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SA09, MPI1_CLK,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SA07, MPI1_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA02, MPI1_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA10, MPI1_DIO3, PIN_NOPULL, 1);


}

void board_pinmux_mpi1_puya_ext(int is64Mb)
{
    HAL_PIN_Set_Analog(PAD_SA04, 1);
    HAL_PIN_Set_Analog(PAD_SA05, 1);
    HAL_PIN_Set_Analog(PAD_SA06, 1);
    HAL_PIN_Set_Analog(PAD_SA08, 1);
    HAL_PIN_Set_Analog(PAD_SA11, 1);
    HAL_PIN_Set_Analog(PAD_SA12, 1);

    if (is64Mb)
    {
        HAL_PIN_Set_Analog(PAD_SA00, 1);
        HAL_PIN_Set(PAD_SA03, MPI1_DIO2, PIN_PULLUP, 1);
    }
    else
    {
        HAL_PIN_Set_Analog(PAD_SA03, 1);
        HAL_PIN_Set(PAD_SA00, MPI1_DIO2, PIN_PULLUP, 1);
    }
}

void board_pinmux_mpi1_gd()
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


void board_pinmux_uart()
{
    HAL_PIN_Set(PAD_PA19, USART1_TXD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA18, USART1_RXD, PIN_PULLUP, 1);
}

int board_boot_from(void)
{
    uint32_t pid = (hwp_hpsys_cfg->IDR & HPSYS_CFG_IDR_PID_Msk) >> HPSYS_CFG_IDR_PID_Pos;
    int r;

    pid &= 7;

    if (pid == BOOT_SIP_PUYA)
        r = BOOT_FROM_SIP_PUYA;
    else if (pid == BOOT_SIP_GD)
        r = BOOT_FROM_SIP_GD;
    else
        r = BOOT_SIP_NONE;
    return r;
}

// Boot MPI Power pins
uint32_t g_boot_opt;
void board_flash_power_on()
{
#ifdef CFG_BOOTROM
    if (__HAL_SYSCFG_GET_REVID() >= HAL_CHIP_REV_ID_A4)
    {
        uint32_t delay;
        g_boot_opt = HAL_Get_backup(RTC_BACKUP_BOOTOPT);

        delay = (g_boot_opt & BOOT_PD_Delay_Msk) >> BOOT_PD_Delay_Pos;
        if (delay)
        {
            BSP_GPIO_Set(MPI_POWER_PIN, 0, 1);
            HAL_Delay_us(delay * 1000);
        }

        BSP_GPIO_Set(MPI_POWER_PIN, 1, 1);
        delay = (g_boot_opt & BOOT_PU_Delay_Msk) >> BOOT_PU_Delay_Pos;
        if (delay)
            HAL_Delay_us(delay * 1000);
    }
    else
        hwp_pmuc->PERI_LDO |= PMUC_PERI_LDO_EN_VDD33_LDO2;
    // No longer needed as bootmode delay is 1s.
    // HAL_Delay_us(0);
    // HAL_Delay_us(2000);
#endif
}



