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

void BSP_PIN_Init(void)
{
#ifdef SOC_BF0_HCPU
    // HCPU pins
#ifdef BSP_USING_PSRAM
#ifdef BSP_ENABLE_MPI1
    // PSRAM
    HAL_PIN_Set(PAD_SA01, MPI1_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA02, MPI1_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA03, MPI1_DIO2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA04, MPI1_DIO3, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA08, MPI1_DIO4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA09, MPI1_DIO5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA10, MPI1_DIO6, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA11, MPI1_DIO7, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA07, MPI1_CLK,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SA06, MPI1_CS,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SA12, MPI1_DQSDM, PIN_PULLDOWN, 1);
#endif
#ifdef BSP_ENABLE_MPI2
    //MPI2 Legacy PSRAM
#ifdef BSP_MPI2_MODE_5
    HAL_PIN_Set(PAD_SB00, MPI2_DM, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB01, MPI2_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB02, MPI2_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB03, MPI2_DIO2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB04, MPI2_DIO3, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB05, MPI2_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SB06, MPI2_CLKB, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SB07, MPI2_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SB08, MPI2_DIO4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB09, MPI2_DIO5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB10, MPI2_DIO6,  PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB11, MPI2_DIO7,  PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB12, MPI2_DQS,   PIN_PULLDOWN, 1);
#elif defined(BSP_MPI2_MODE_3)
    // MPI2 Xceela PSRAM
    HAL_PIN_Set(PAD_SB01, MPI2_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB02, MPI2_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB03, MPI2_DIO2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB04, MPI2_DIO3, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB05, MPI2_DIO4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB06, MPI2_DIO5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB07, MPI2_DIO6, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB08, MPI2_DIO7, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB11, MPI2_CLK,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SB12, MPI2_CS,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SB09, MPI2_DQSDM, PIN_PULLDOWN, 1);
#endif /*BSP_MPI2_MODE_3*/
#endif
#endif

    HAL_PIN_Set(PAD_PA72, GPIO_A72, PIN_PULLUP, 1);               // SB_EN/MPI2_EN
    HAL_PIN_Set(PAD_PA74, GPIO_A74, PIN_PULLUP, 1);               // SA_EN/MPI1_EN

    HAL_PIN_Set(PAD_PA00, WLAN_ACTIVE, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_PA01, I2C1_SCL, PIN_PULLDOWN, 1);              // CTP_I2C_SCL
    HAL_PIN_Set(PAD_PA02, I2C1_SDA,  PIN_PULLDOWN, 1);             // CTP_I2C_SDA

    HAL_PIN_Set(PAD_PA03, USART1_TXD,  PIN_PULLUP, 1);             // CAN1_TXD
    HAL_PIN_Set(PAD_PA04, USART1_RXD,  PIN_PULLUP, 1);             // CAN1_RXD

#ifdef BSP_ENABLE_MPI3
    // MPI3
    HAL_PIN_Set(PAD_PA06, MPI3_CS,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA07, MPI3_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA08, MPI3_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA09, MPI3_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA10, MPI3_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA11, MPI3_DIO3, PIN_PULLUP, 1);
#endif

#ifdef BSP_USING_SDIO
    {
        HAL_PIN_Set(PAD_PA27, SD1_CMD, PIN_PULLUP, 1);
        HAL_Delay_us(20);   // add a delay before clock setting to avoid wrong cmd happen
        HAL_PIN_Set(PAD_PA26, SD1_CLK, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_PA22, SD1_DIO0, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA15, SD1_DIO1, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA12, SD1_DIO2, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA20, SD1_DIO3, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA05, GPIO_A5,  PIN_PULLUP, 1);   // SD1_EN
    }
#endif


#ifdef BSP_USING_USBD
    {
        HAL_PIN_Set_Analog(PAD_PA17, 1);
        HAL_PIN_Set_Analog(PAD_PA18, 1);
    }
#endif

    // PAD_PA13-PA47 for LCDs, following is default QSPI selection
    {
        HAL_PIN_Set(PAD_PA31, GPIO_A31, PIN_NOPULL, 1);         // LCD_VIO_EN
        HAL_PIN_Set(PAD_PA33, LCDC1_SPI_TE, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA35, GPIO_A35, PIN_PULLUP, 1);         // LCD backlight PWM
        HAL_PIN_Set(PAD_PA36, LCDC1_SPI_CS, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA37, LCDC1_SPI_CLK, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA38, LCDC1_SPI_DIO0, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA39, LCDC1_SPI_DIO1, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA40, LCDC1_SPI_DIO2, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA41, LCDC1_SPI_DIO3, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA47, GPIO_A47, PIN_PULLDOWN, 1);       // LCDC1 VDD_EN
    }

    HAL_PIN_Set(PAD_PA48, GPIO_A48, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA49, GPIO_A49, PIN_NOPULL, 1);

    HAL_PIN_Set(PAD_PA50, GPIO_A50, PIN_PULLDOWN, 1);       // LCD_RESETB
    HAL_PIN_Set(PAD_PA51, GPIO_A51, PIN_PULLDOWN, 1);       // #CTP_INT

    // PAD_PA52-PA54 for GPIO
    HAL_PIN_Set(PAD_PA52, GPIO_A52, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA53, GPIO_A53, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA54, GPIO_A54, PIN_PULLDOWN, 1);

    // PAD_PA55 XTAL32K_XI
    // PAD_PA56 XTAL32K_XO

    // PAD_PA57-PA78 for GPIO
    HAL_PIN_Set(PAD_PA52, GPIO_A52, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA53, GPIO_A53, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA54, GPIO_A54, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA55, GPIO_A55, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA56, GPIO_A56, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA57, GPIO_A57, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA58, GPIO_A58, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA59, GPIO_A59, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA60, GPIO_A60, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA61, GPIO_A61, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA62, GPIO_A62, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA63, GPIO_A63, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA64, GPIO_A64, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA65, GPIO_A65, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA66, GPIO_A66, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA67, GPIO_A67, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA68, GPIO_A68, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA69, GPIO_A69, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA70, GPIO_A70, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA71, GPIO_A71, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA72, GPIO_A72, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA73, GPIO_A73, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA74, GPIO_A74, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA75, GPIO_A75, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA76, GPIO_A76, PIN_PULLDOWN, 1);     // WLAN_INT
    HAL_PIN_Set(PAD_PA77, GPIO_A77, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA78, GPIO_A78, PIN_PULLDOWN, 1);

#endif
    //B0-B12 GPIO
    HAL_PIN_Set(PAD_PB00, GPIO_B0, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB01, GPIO_B1, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB02, GPIO_B2, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB03, GPIO_B3, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB04, GPIO_B4, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB05, GPIO_B5, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB06, GPIO_B6, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB07, GPIO_B7, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB08, GPIO_B8, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB09, GPIO_B9, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB10, GPIO_B10, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB11, GPIO_B11, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB12, GPIO_B12, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB14, GPIO_B14, PIN_PULLDOWN, 0);

    // PAD_B13 SWDIO
    // PAD_B15 SWDCLK

    {
        HAL_PIN_Set(PAD_PB16, USART4_RXD,  PIN_PULLUP, 0);  // UART4, LCPU default console
        HAL_PIN_Set(PAD_PB17, USART4_TXD,  PIN_PULLUP, 0);
    }

    //B18-B21 GPIO
    HAL_PIN_Set(PAD_PB18, GPIO_B18, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB19, GPIO_B19, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB20, GPIO_B20, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB21, GPIO_B21, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB22, GPIO_B22, PIN_PULLDOWN, 0);       // LCD_PWR_EN
#if defined(LCD_USING_PWM_AS_BACKLIGHT)
    HAL_PIN_Set(PAD_PB23, GPTIM3_CH4, PIN_NOPULL, 0);   // LCDC1_BL_PWM_CTRL, LCD backlight PWM
#else
    HAL_PIN_Set(PAD_PB23, GPIO_B23, PIN_NOPULL, 0);     // LCDC1_BL_PWM_CTRL, LCD backlight PWM
#endif
    HAL_PIN_Set(PAD_PB24, GPIO_B24, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB25, GPIO_B25, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB26, GPIO_B26, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB27, GPIO_B27, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB28, GPIO_B28, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB29, GPIO_B29, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB30, GPIO_B30, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB31, GPIO_B31, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB32, GPIO_B32, PIN_PULLDOWN, 0);       // PWR_KEY
    HAL_PIN_Set(PAD_PB33, GPIO_B33, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB34, GPIO_B34, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB35, GPIO_B35, PIN_PULLDOWN, 0);       // CTP_RESET
    HAL_PIN_Set(PAD_PB36, GPIO_B36, PIN_PULLDOWN, 0);
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/