/**
  ******************************************************************************
  * @file   drv_io.c
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
    int pad;
    pin_function func;

#ifdef SOC_BF0_HCPU

#ifdef HDK_U4O5
    HAL_PIN_Set(PAD_SIP01, PSRAM_DQ0, PIN_PULLDOWN, 1);           // OPI PSRAM
    HAL_PIN_Set(PAD_SIP02, PSRAM_DQ1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SIP03, PSRAM_DQ2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SIP04, PSRAM_DQ3, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SIP08, PSRAM_DQ4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SIP09, PSRAM_DQ5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SIP10, PSRAM_DQ6, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SIP11, PSRAM_DQ7, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SIP12, PSRAM_DQS0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SIP07, PSRAM_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SIP05, PSRAM_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SIP06, PSRAM_CLKB, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SIP00, PSRAM_DM0, PIN_NOPULL, 1);
#endif

    HAL_PIN_Set(PAD_PA01, GPIO_A1, PIN_PULLDOWN, 1);              //BT, external BT power
    HAL_PIN_Set(PAD_PA03, GPIO_A3, PIN_PULLDOWN, 1);
    HAL_PIN_SetMode(PAD_PA03, 1, PIN_DIGITAL_O_PULLUP);
    HAL_PIN_Set(PAD_PA10, I2C1_SCL, PIN_NOPULL, 1);             // I2C1 scl, touch screen
    HAL_PIN_Set(PAD_PA14, I2C1_SDA, PIN_NOPULL, 1);             // I2C1 sda, touch screen
    HAL_PIN_Set(PAD_PA20, LCDC1_SPI_CLK, PIN_NOPULL, 1);        // LCDC 1  QAD-SPI mode
    HAL_PIN_Set(PAD_PA31, LCDC1_SPI_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA34, LCDC1_SPI_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA36, LCDC1_SPI_DIO1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, LCDC1_SPI_DIO2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA42, LCDC1_SPI_DIO3, PIN_NOPULL, 1);
#ifndef PMIC_CTRL_ENABLE
    HAL_PIN_Set(PAD_PA44, GPIO_A44, PIN_NOPULL, 1);            // MOTOR_EN (hig-->open)
#else
    HAL_PIN_Set(PAD_PA44, GPIO_A44, PIN_PULLDOWN, 1);          //sda
#endif
    HAL_PIN_Set(PAD_PA45, GPIO_A45, PIN_NOPULL, 1);            // MOTOR PWM
    HAL_PIN_SetMode(PAD_PA45, 1, PIN_DIGITAL_O_NORMAL);
    HAL_PIN_Set(PAD_PA47, GPIO_A47, PIN_PULLDOWN, 1);          // I2C1_RESET
    HAL_PIN_Set(PAD_PA49, DBG_DO8, PIN_NOPULL, 1);             // UART1 TX
    HAL_PIN_Set(PAD_PA51, USART1_RXD, PIN_PULLUP, 1);          // UART1 RX
#ifndef PMIC_CTRL_ENABLE
    HAL_PIN_Set(PAD_PA55, GPIO_A55, PIN_NOPULL, 1);            // LCD Power, LCDC1_SPI_EN (high-->open)
#else
    HAL_PIN_Set(PAD_PA55, GPIO_A55, PIN_PULLDOWN, 1);          // sdl
#endif
    HAL_PIN_Set(PAD_PA58, GPIO_A58, PIN_NOPULL, 1);            // QSPI2_EN (high-->open, boot rom auto set high)
    HAL_PIN_Set(PAD_PA60, QSPI2_CLK, PIN_PULLUP, 1);           // SPI2
    HAL_PIN_Set(PAD_PA61, QSPI2_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA63, QSPI2_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA65, QSPI2_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA66, QSPI2_DIO2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA68, QSPI2_DIO3, PIN_NOPULL, 1);
#if defined(LCD_USING_PWM_AS_BACKLIGHT)
    HAL_PIN_Set(PAD_PA70, GPTIM1_CH4, PIN_NOPULL, 1);
#else
    HAL_PIN_Set(PAD_PA70, GPIO_A70, PIN_NOPULL, 1);            // LCD backlight
    HAL_PIN_SetMode(PAD_PA70, 1, PIN_DIGITAL_O_NORMAL);
#endif
    HAL_PIN_Set(PAD_PA77, LCDC1_SPI_TE, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA78, GPIO_A78, PIN_NOPULL, 1);            // LCD rest pin
    HAL_PIN_Set(PAD_PA79, GPIO_A79, PIN_NOPULL, 1);            // I2C1_INT, touch screen
    HAL_PIN_Set(PAD_PA80, GPIO_A80, PIN_NOPULL, 1);            // external BT interrupt input
#endif
    HAL_PIN_Set(PAD_PB01, GPIO_B1,  PIN_NOPULL, 0);            // SPI3_EN
    HAL_PIN_Set(PAD_PB03, GPIO_B3,  PIN_NOPULL, 0);            // HR_EN
    HAL_PIN_Set(PAD_PB04, I2C4_SCL, PIN_NOPULL, 0);            // I2C4 (Heart rate sensor)
    HAL_PIN_Set(PAD_PB05, I2C4_SDA, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB08, GPIO_B8,  PIN_NOPULL, 0);            // calib
    HAL_PIN_Set_Analog(PAD_PB10, 0);                           // Battery Voltage ADC

    HAL_PIN_Set(PAD_PB13, SPI3_CLK, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB16, SPI3_DO,  PIN_PULLDOWN, 0);          // SPI3 (GSensor)
    HAL_PIN_Set(PAD_PB19, SPI3_DI,  PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB23, SPI3_CS,  PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB24, GPIO_B24, PIN_PULLUP, 0);            // CHG_IND
    HAL_PIN_Set(PAD_PB25, GPIO_B25, PIN_NOPULL, 0);            // rotate key 1
    HAL_PIN_Set(PAD_PB29, GPIO_B29, PIN_NOPULL, 0);            // rotate key 2
    //PAD_PB41                                                 // swclk
    //PAD_PB42                                                 // swdio
    HAL_PIN_Set(PAD_PB43, GPIO_B43, PIN_NOPULL, 0);            // I2C4_INT
    HAL_PIN_Set(PAD_PB44, GPIO_B44, PIN_NOPULL, 0);            // SPI3_INT
    HAL_PIN_Set(PAD_PB45, USART3_TXD, PIN_NOPULL, 0);          // uart3
    HAL_PIN_Set(PAD_PB46, USART3_RXD, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB47, GPIO_B47, PIN_NOPULL, 0);            // CHG_DET
    HAL_PIN_Set(PAD_PB48, GPIO_B48, PIN_NOPULL, 0);            //FPC_KEY, wakeup key
    HAL_PIN_SetMode(PAD_PB25, 0, PIN_DIGITAL_IO_NORMAL);
    HAL_PIN_SetMode(PAD_PB29, 0, PIN_DIGITAL_IO_NORMAL);
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
