/**
  ******************************************************************************
  * @file   bsp_pinmux.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
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
#ifdef BSP_GPADC_SUPPORT_MULTI_CH_SAMPLING
void BSP_adc_pinmux_config()
{
    HAL_PIN_Set(PAD_PA28, GPIO_A28, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA29, GPIO_A29, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA30, GPIO_A30, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA31, GPIO_A31, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA32, GPIO_A32, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA33, GPIO_A33, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA34, GPIO_A34, PIN_NOPULL, 1);

    /*adc1_ch0*/
    HAL_PIN_Set_Analog(PAD_PA28, 1);
    //HAL_PIN_SetMode(PAD_PA28, 1, PIN_ANALOG_INPUT);

    /*adc1_ch1*/
    HAL_PIN_Set_Analog(PAD_PA29, 1);
    //HAL_PIN_SetMode(PAD_PA29, 1, PIN_ANALOG_INPUT);

    /*adc1_ch2*/
    HAL_PIN_Set_Analog(PAD_PA30, 1);
    //HAL_PIN_SetMode(PAD_PA30, 1, PIN_ANALOG_INPUT);

    /*adc1_ch3*/
    HAL_PIN_Set_Analog(PAD_PA31, 1);
    //HAL_PIN_SetMode(PAD_PA31, 1, PIN_ANALOG_INPUT);

    /*adc1_ch4*/
    HAL_PIN_Set_Analog(PAD_PA32, 1);
    //HAL_PIN_SetMode(PAD_PA32, 1, PIN_ANALOG_INPUT);

    /*adc1_ch5*/
    HAL_PIN_Set_Analog(PAD_PA33, 1);
    //HAL_PIN_SetMode(PAD_PA33, 1, PIN_ANALOG_INPUT);

}
#endif
void BSP_PIN_Init(void)
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

#ifdef BSP_USING_LCD
    HAL_PIN_Set(PAD_PA04, LCDC1_SPI_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA03, LCDC1_SPI_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA05, LCDC1_SPI_DIO0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA06, LCDC1_SPI_DIO1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA07, LCDC1_SPI_DIO2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA08, LCDC1_SPI_DIO3, PIN_NOPULL, 1);
#endif
#ifdef BSP_LCDC_USING_DDR_QADSPI
    if (__HAL_SYSCFG_GET_REVID() < 3)
    {
        HAL_PIN_Set_DS0(PAD_PA04, 1, 1);
        HAL_PIN_Set_DS0(PAD_PA05, 1, 0);
        HAL_PIN_Set_DS0(PAD_PA06, 1, 0);
        HAL_PIN_Set_DS0(PAD_PA07, 1, 0);
        HAL_PIN_Set_DS0(PAD_PA08, 1, 0);

        HAL_PIN_Set_DS1(PAD_PA04, 1, 1);
        HAL_PIN_Set_DS1(PAD_PA05, 1, 0);
        HAL_PIN_Set_DS1(PAD_PA06, 1, 0);
        HAL_PIN_Set_DS1(PAD_PA07, 1, 0);
        HAL_PIN_Set_DS1(PAD_PA08, 1, 0);
    }
#endif /* DDR_LCD */

#ifdef BSP_USING_LCD
    HAL_PIN_Set(PAD_PA02, LCDC1_SPI_TE, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA00, GPIO_A0, PIN_NOPULL, 1);

    //HAL_PIN_Set(PAD_PA00, LCDC1_SPI_RSTB, PIN_PULLUP, 1);
#if defined(LCD_USING_PWM_AS_BACKLIGHT)
    HAL_PIN_Set(PAD_PA01, GPTIM2_CH4, PIN_NOPULL, 1);   // LCDC1_BL_PWM_CTRL, LCD backlight PWM
#else
    HAL_PIN_Set(PAD_PA01, GPIO_A1, PIN_NOPULL, 1);     // LCDC1_BL_PWM_CTRL, LCD backlight PWM
#endif
#endif

    // Key1
    /* Keep default pull-down unchanged. Uart download driver would use this function,
     * if pulldown is disabled, download driver would not work on the board without external pull-down
     */
    // HAL_PIN_Set(PAD_PA34, GPIO_A34, PIN_NOPULL, 1);

#ifdef BSP_USING_I2C3
    /* MAG */
    HAL_PIN_Set(PAD_PA29, I2C3_SCL, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA28, I2C3_SDA, PIN_PULLUP, 1);
#endif

    //HAL_PIN_Set(PAD_PB00, USART4_TXD, PIN_PULLUP, 0);
    //HAL_PIN_Set(PAD_PB01, USART4_RXD, PIN_PULLUP, 0);

#if defined(BSP_USING_USBD) || defined(BSP_USING_USBH)
    HAL_PIN_Set_Analog(PAD_PA35, 1);
    HAL_PIN_Set_Analog(PAD_PA36, 1);
    HAL_PIN_Set(PAD_PA32, GPIO_A32, PIN_NOPULL, 1);//USB detection pin
#else
    HAL_PIN_Set(PAD_PA35, USART3_TXD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA36, USART3_RXD, PIN_PULLUP, 1);
#endif

#endif

#ifdef RT_USING_AUDIO
    HAL_PIN_Set(PAD_PA09, GPIO_A9, PIN_NOPULL, 1);      //share with MIC
    HAL_PIN_Set(PAD_PA10, GPIO_A10, PIN_NOPULL, 1);     // AUDIO_PA_CTRL
#endif
    HAL_PIN_Set(PAD_PA11, GPIO_A11, PIN_NOPULL, 1);     // VDD_EN

    /* Motor PWM */
    HAL_PIN_Set(PAD_PA20, GPTIM1_CH4, PIN_NOPULL, 1);

#if defined(LXT_DISABLE) && defined(BSP_USING_PDM1)
    HAL_PIN_Set(PAD_PA22, PDM1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA23, PDM1_DATA, PIN_PULLDOWN, 1);
    HAL_PIN_SetMode(PAD_PA23, 1, PIN_DIGITAL_IO_PULLDOWN);
#endif
    HAL_PIN_Set(PAD_PA24, GPIO_A24, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA25, GPIO_A25, PIN_NOPULL, 1);

    HAL_PIN_Set(PAD_PA30, GPIO_A30, PIN_NOPULL, 1);
    HAL_PIN_SetMode(PAD_PA30, 1, PIN_DIGITAL_IO_NORMAL);

    /* MAG EN */
    HAL_PIN_Set(PAD_PA31, GPIO_A31, PIN_NOPULL, 1);
    HAL_PIN_SetMode(PAD_PA31, 1, PIN_DIGITAL_O_NORMAL);

    /* HR WKUP INT */
    HAL_PIN_Set(PAD_PA37, GPIO_A37, PIN_NOPULL, 1);

    /* HR RST */
    HAL_PIN_Set(PAD_PA38, GPIO_A38, PIN_NOPULL, 1);

    // GS/HR
    HAL_PIN_Set(PAD_PA40, I2C2_SCL, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA39, I2C2_SDA, PIN_PULLUP, 1);

    // Touch
    HAL_PIN_Set(PAD_PA42, I2C1_SCL, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA41, I2C1_SDA, PIN_PULLUP, 1);

    /* TP WKUP INT */
    HAL_PIN_Set(PAD_PA43, GPIO_A43, PIN_NOPULL, 1);
    /* TP RESET */
    HAL_PIN_Set(PAD_PA44, GPIO_A44, PIN_NOPULL, 1);

    // LCPU pins
#ifdef BSP_GPADC_SUPPORT_MULTI_CH_SAMPLING
    BSP_adc_pinmux_config();
#endif
    // UART4
    //HAL_PIN_Set(PAD_PB01, USART4_TXD, PIN_PULLUP, 0);
    //HAL_PIN_Set(PAD_PB00, USART4_RXD, PIN_PULLUP, 0);
    //HAL_PIN_Set(PAD_PB18, USART5_TXD, PIN_PULLUP, 0);
    //HAL_PIN_Set(PAD_PB17, USART5_RXD, PIN_PULLUP, 0);

    // UART5
    //HAL_PIN_Set(PAD_PB03, USART5_TXD, PIN_PULLUP, 0);
    //HAL_PIN_Set(PAD_PB02, USART5_RXD, PIN_PULLUP, 0);

}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
