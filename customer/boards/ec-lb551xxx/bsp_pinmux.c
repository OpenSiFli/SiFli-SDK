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

#ifndef TOUCH_RESET_PIN
    #define TOUCH_RESET_PIN (44)
#endif

void BSP_PIN_Init(void)
{
    int pad;
    pin_function func;

#ifdef SOC_BF0_HCPU

#ifndef LB551_U8N5
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

    HAL_PIN_Set(TOUCH_RESET_PIN + PAD_PA00, TOUCH_RESET_PIN + GPIO_A0, PIN_NOPULL, 1); // I2C1(Touch) Reset GPIO A44 if version 2,GPIO A1 if version 1
    HAL_PIN_Set(PAD_PA03, GPIO_A3, PIN_NOPULL, 1);              // USB_DM/PDM1_DATA/TP_EN

    HAL_PIN_Set(PAD_PA10, I2C1_SCL, PIN_PULLUP, 1);             // I2C1(Touch)
    HAL_PIN_Set(PAD_PA14, I2C1_SDA, PIN_PULLUP, 1);

#ifdef BSP_LCDC_USING_DBI
    HAL_PIN_Set(PAD_PA20, LCDC1_8080_WR, PIN_NOPULL, 1);        // LCDC 1 DBI8080 8bit mode
    HAL_PIN_Set(PAD_PA31, LCDC1_8080_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA34, LCDC1_8080_RD, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA36, LCDC1_8080_DC, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, LCDC1_8080_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA42, LCDC1_8080_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA77, LCDC1_8080_TE, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA78, LCDC1_8080_RSTB, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA79, GPIO_A79, PIN_NOPULL, 1);             // LCD Power, LCDC_SPI_EN
#elif defined(BSP_LCDC_USING_JDI_PARALLEL)
    HAL_PIN_Set(PAD_PA20, LCDC1_JDI_VCK, PIN_NOPULL, 1);        // LCDC 1 JDI parallel mode
    HAL_PIN_Set(PAD_PA31, LCDC1_JDI_VST, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA34, LCDC1_JDI_XRST, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA36, LCDC1_JDI_HCK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, LCDC1_JDI_HST, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA42, LCDC1_JDI_ENB, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA77, LCDC1_JDI_G2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA78, LCDC1_JDI_B1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA79, LCDC1_JDI_B2, PIN_NOPULL, 1);
#else
    HAL_PIN_Set(PAD_PA20, LCDC1_SPI_CLK, PIN_NOPULL, 1);        // LCDC 1  QAD-SPI mode
    HAL_PIN_Set(PAD_PA31, LCDC1_SPI_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA34, LCDC1_SPI_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA36, LCDC1_SPI_DIO1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, LCDC1_SPI_DIO2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA42, LCDC1_SPI_DIO3, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA77, LCDC1_SPI_TE, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA78, GPIO_A78, PIN_NOPULL, 1);            //LCD reset pin
    HAL_PIN_Set(PAD_PA79, GPIO_A79, PIN_NOPULL, 1);             // LCD Power, LCDC_SPI_EN
#endif /* BSP_LCDC_USING_DBI */

#ifdef BSP_ENABLE_QSPI3
    HAL_PIN_Set(PAD_PA44, QSPI3_CLK, PIN_NOPULL, 1);            // QSPI3
    HAL_PIN_Set(PAD_PA45, QSPI3_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA47, QSPI3_DIO0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA49, QSPI3_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA51, QSPI3_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA55, QSPI3_DIO3, PIN_PULLUP, 1);
#elif defined(BSP_USING_SDIO)
    HAL_PIN_Set(PAD_PA44, SD2_CLK, PIN_NOPULL, 1);            // QSPI3
    HAL_PIN_Set(PAD_PA45, SD2_CMD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA47, SD2_DIO0, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA49, SD2_DIO1, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA51, SD2_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA55, SD2_DIO3, PIN_PULLUP, 1);
#elif defined(BSP_LCDC_USING_DBI)
    HAL_PIN_Set(PAD_PA44, LCDC1_8080_DIO2, PIN_PULLDOWN, 1);            // LCDC 1 DBI8080 8bit mode
    HAL_PIN_Set(PAD_PA45, LCDC1_8080_DIO3, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA47, LCDC1_8080_DIO4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA49, LCDC1_8080_DIO5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA51, LCDC1_8080_DIO6, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA55, LCDC1_8080_DIO7, PIN_PULLDOWN, 1);
#elif defined(BSP_LCDC_USING_JDI_PARALLEL)
    HAL_PIN_Set(PAD_PA44, LCDC1_JDI_FRP, PIN_PULLDOWN, 1);            // LCDC 1 JDI parallel mode
    HAL_PIN_Set(PAD_PA45, LCDC1_JDI_XFRP, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA47, LCDC1_JDI_VCOM, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA49, LCDC1_JDI_R1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA51, LCDC1_JDI_R2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA55, LCDC1_JDI_G1, PIN_PULLDOWN, 1);
#elif defined(BSP_QSPI2_DUAL_CHIP)
    HAL_PIN_Set(PAD_PA47, QSPI2_DIO4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA49, QSPI2_DIO5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA51, QSPI2_DIO6, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA55, QSPI2_DIO7, PIN_PULLUP, 1);
#else
    HAL_PIN_Set(PAD_PA49, DBG_DO8, PIN_NOPULL, 1);              // DBG_DO8 is used for USART1_TXD
    HAL_PIN_Set(PAD_PA51, USART1_RXD, PIN_PULLUP, 1);

#if LB551_VERSION==2
    HAL_PIN_Set(TOUCH_IRQ_PIN + PAD_PA00, TOUCH_IRQ_PIN + GPIO_A0, PIN_NOPULL, 1); // I2C1(Touch) interrupt GPIO A45
#endif

#endif


    HAL_PIN_Set(PAD_PA58, GPIO_A58, PIN_NOPULL, 1);             // QSPI2/QSPI3 Power
    HAL_PIN_Set(PAD_PA60, QSPI2_CLK, PIN_NOPULL, 1);            // QSPI2
    HAL_PIN_Set(PAD_PA61, QSPI2_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA63, QSPI2_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA65, QSPI2_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA66, QSPI2_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA68, QSPI2_DIO3, PIN_PULLUP, 1);

    HAL_PIN_Set(PAD_PA70, GPIO_A70, PIN_NOPULL, 1);           // LCD backlight

    //HAL_PIN_Set(PAD_PA80, GPTIM2_CH3, PIN_NOPULL, 1);           // Motor PWM
#endif

    //HAL_PIN_Set(PAD_PB01, GPIO_B1, PIN_PULLUP, 0);              // I2C4 INT
    //HAL_PIN_Set(PAD_PB03, GPIO_B3, PIN_NOPULL, 0);              // I2C4 power
    HAL_PIN_Set(PAD_PB04, I2C4_SCL, PIN_PULLUP, 0);             // I2C4 (Heart rate sensor)
    HAL_PIN_Set(PAD_PB05, I2C4_SDA, PIN_PULLUP, 0);

    HAL_PIN_Set_Analog(PAD_PB08, 0);                            // Temperature ADC
    HAL_PIN_Set_Analog(PAD_PB10, 0);                            // Battery Voltage ADC

    HAL_PIN_Set(PAD_PB13, SPI3_CLK, PIN_PULLDOWN, 0);             // SPI3 (GSensor)
    HAL_PIN_Set(PAD_PB16, SPI3_DO, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB19, SPI3_DI, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB23, SPI3_CS, PIN_PULLUP, 0);

    //HAL_PIN_Set(PAD_PB24, GPIO_B24, PIN_NOPULL, 0);             // SYS_EN/I2C5_INT/PMIC control
    HAL_PIN_Set(PAD_PB25, GPIO_B25, PIN_NOPULL, 0);             // SPI3_EN
#if defined(BSP_USING_SDADC)
    HAL_PIN_Select(PAD_PB25, 10, 0);
    HAL_PIN_SetMode(PAD_PB25, 0, PIN_ANALOG_INPUT);
#endif
#if LB551_VERSION!=2
    HAL_PIN_Set(PAD_PB29, GPIO_B29, PIN_NOPULL, 0); // I2C1(Touch) interrupt GPIO B29
#endif
    // PB31 SWCLK,
    // PB34 SWDIO
    HAL_PIN_Set(PAD_PB43, I2C5_SCL, PIN_PULLUP, 0);             // I2C5(PMIC control)
    HAL_PIN_Set(PAD_PB44, I2C5_SDA, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB45, USART3_TXD, PIN_NOPULL, 0);           // USART3 TX/SPI3_INT
    HAL_PIN_Set(PAD_PB46, USART3_RXD, PIN_PULLUP, 0);           // USART3 RX
    /* avoid current leak */
    HAL_PIN_Set(PAD_PB47, GPIO_B47, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB48, GPIO_B48, PIN_NOPULL, 0);             // Key1
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
