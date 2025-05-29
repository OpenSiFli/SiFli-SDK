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

static void BSP_PIN_Common(void)
{
#ifdef SOC_BF0_HCPU
    // HCPU pins

    uint32_t pid = (hwp_hpsys_cfg->IDR & HPSYS_CFG_IDR_PID_Msk) >> HPSYS_CFG_IDR_PID_Pos;

    pid &= 7;

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

#ifdef BSP_ENABLE_MPI2
    // MPI2
    HAL_PIN_Set(PAD_PA16, MPI2_CLK,  PIN_NOPULL,   1);
    HAL_PIN_Set(PAD_PA12, MPI2_CS,   PIN_NOPULL,   1);
    HAL_PIN_Set(PAD_PA15, MPI2_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA13, MPI2_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA14, MPI2_DIO2, PIN_PULLUP,   1);
    HAL_PIN_Set(PAD_PA17, MPI2_DIO3, PIN_PULLUP, 1);
#elif BSP_USING_SDIO
    HAL_PIN_Set(PAD_PA15, SD1_CMD, PIN_PULLUP, 1);
    HAL_Delay_us(20);
    HAL_PIN_Set(PAD_PA12, SD1_DIO2,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA13, SD1_DIO3, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA14, SD1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA16, SD1_DIO0, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA17, SD1_DIO1, PIN_PULLUP, 1);
#endif
    HAL_PIN_Set(PAD_PA00, GPIO_A0,  PIN_PULLDOWN, 1);     // #LCD_RESETB
    HAL_PIN_Set(PAD_PA10, GPIO_A10, PIN_PULLDOWN, 1);     // AUDIO_PA_CTRL

    // UART1 - debug
    HAL_PIN_Set(PAD_PA18, USART1_RXD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA19, USART1_TXD, PIN_PULLUP, 1);

    // UART2 - log
    HAL_PIN_Set(PAD_PA20, USART2_RXD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA27, USART2_TXD, PIN_PULLUP, 1);

    // Key1 - Power key
    /* Keep default pull-down unchanged. Uart download driver would use this function,
     * if pulldown is disabled, download driver would not work on the board without external pull-down
     */
    // HAL_PIN_Set(PAD_PA34, GPIO_A34, PIN_NOPULL, 1);
    // Key2
    HAL_PIN_Set(PAD_PA11, GPIO_A11, PIN_NOPULL, 1);

    // PA22 #XTAL32K_XI
    // PA23 #XTAL32K_XO

    // USBD
    HAL_PIN_Set_Analog(PAD_PA35, 1);                    // USB_DP
    HAL_PIN_Set_Analog(PAD_PA36, 1);                    // USB_DM

    // SPI1(TF card)
    HAL_PIN_Set(PAD_PA24, SPI1_DIO, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA25, SPI1_DI,  PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA28, SPI1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA29, SPI1_CS,  PIN_NOPULL, 1);

//     HAL_PIN_Set_DS0(PAD_PA24, 1, 1);
//     HAL_PIN_Set_DS0(PAD_PA25, 1, 1);
//     HAL_PIN_Set_DS0(PAD_PA28, 1, 1);
//     HAL_PIN_Set_DS0(PAD_PA29, 1, 1);
//
//     HAL_PIN_Set_DS1(PAD_PA24, 1, 1);
//     HAL_PIN_Set_DS1(PAD_PA25, 1, 1);
//     HAL_PIN_Set_DS1(PAD_PA28, 1, 1);
//     HAL_PIN_Set_DS1(PAD_PA29, 1, 1);
#if defined(BSP_USING_PWM3) || defined(BSP_USING_RGBLED_WITCH_PWM3)
    HAL_PIN_Set(PAD_PA32, GPTIM2_CH1, PIN_PULLUP, 1);   // RGB LED
#endif
    // GPIOs
    HAL_PIN_Set(PAD_PA21, GPIO_A21, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA26, GPIO_A26, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA32, GPIO_A32, PIN_PULLDOWN, 1);   // RGB LED
    HAL_PIN_Set(PAD_PA38, GPIO_A38, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA44, GPIO_A44, PIN_PULLDOWN, 1);   // VBUS_DET
#endif

}

void BSP_PIN_Touch(void)
{
    // Touch
    HAL_PIN_Set(PAD_PA00 + TOUCH_IRQ_PIN, GPIO_A0 + TOUCH_IRQ_PIN, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA00 + TOUCH_RESET_PIN, GPIO_A0 + TOUCH_RESET_PIN, PIN_PULLDOWN, 1);
}


void BSP_PIN_LCD(void)
{
    //edp gpio pin
    const uint32_t pin_out[] =
    {
        TPS_WAKEUP,
        TPS_PWRCOM,
        TPS_PWRUP,
        EPD_LE,
        EPD_OE,
        EPD_STV,
        EPD_CPV,
        EPD_GMODE,
    };
    int pin_num = sizeof(pin_out) / sizeof(pin_out[0]);

    //epd pin init
    for (int i = 0; i < pin_num; i++)
    {
        HAL_PIN_Set(PAD_PA00 + pin_out[i], GPIO_A0 + pin_out[i], PIN_NOPULL, 1);
    }

    HAL_RCC_EnableModule(RCC_MOD_GPIO1); // GPIO clock enable
    GPIO_InitTypeDef GPIO_InitStruct;

    for (int i = 0; i < pin_num; i++)
    {
        GPIO_InitStruct.Pin = pin_out[i];
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);
        HAL_GPIO_WritePin(hwp_gpio1, pin_out[i], GPIO_PIN_RESET);
    }


    HAL_PIN_Set(PAD_PA00 + EPD_CLK, LCDC1_8080_WR, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA00 + EPD_SPH, LCDC1_8080_DC, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA00 + EPD_D0,  LCDC1_8080_DIO0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA00 + EPD_D1,  LCDC1_8080_DIO1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA00 + EPD_D2,  LCDC1_8080_DIO2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA00 + EPD_D3,  LCDC1_8080_DIO3, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA00 + EPD_D4,  LCDC1_8080_DIO4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA00 + EPD_D5,  LCDC1_8080_DIO5, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA00 + EPD_D6,  LCDC1_8080_DIO6, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA00 + EPD_D7,  LCDC1_8080_DIO7, PIN_NOPULL, 1);


    HAL_PIN_Set(PAD_PA00 + TPS_SCL, I2C1_SCL, PIN_PULLUP, 1); // i2c io select
    HAL_PIN_Set(PAD_PA00 + TPS_SDA, I2C1_SDA, PIN_PULLUP, 1);

    // key init
    HAL_PIN_Set(PAD_PA00 + EPD_KEY1, GPIO_A0 + EPD_KEY1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA00 + EPD_KEY2, GPIO_A0 + EPD_KEY2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA00 + EPD_KEY3, GPIO_A0 + EPD_KEY3, PIN_PULLDOWN, 1);

}

void BSP_PIN_Init(void)
{
    BSP_PIN_Common();

    BSP_PIN_LCD();

}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
