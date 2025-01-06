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

#ifdef SOC_BF0_HCPU



#ifdef BSP_USING_EXT_PSRAM
    HAL_PIN_Set(PAD_PA28, PSRAM_DQ0, PIN_PULLDOWN, 1);           // OPI PSRAM External, XCEELA interface, 16MB on board.
    HAL_PIN_Set(PAD_PA29, PSRAM_DQ1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA30, PSRAM_DQ2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA31, PSRAM_DQ3, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA34, PSRAM_DQ4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA36, PSRAM_DQ5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA38, PSRAM_DQ6, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA42, PSRAM_DQ7, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA35, PSRAM_DQS0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA20, PSRAM_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA37, PSRAM_CS, PIN_NOPULL, 1);
#else
    HAL_PIN_Set(PAD_SIP01, PSRAM_DQ0, PIN_PULLDOWN, 1);           // SIP OPI PSRAM, legacy interface
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

    HAL_PIN_Set(PAD_SIP06, PSRAM_CLKB, PIN_NOPULL, 1);          // Legacy interface only.
    HAL_PIN_Set(PAD_SIP00, PSRAM_DM0, PIN_NOPULL, 1);
#endif

    HAL_PIN_Set(PAD_PA00, GPIO_A0, PIN_NOPULL, 1);              //Touch screen reset

#ifdef USB_DEBUG
    HAL_PIN_Set(PAD_PA01, GPIO_A1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA03, GPIO_A3, PIN_NOPULL, 1);
#else
    HAL_PIN_Set(PAD_PA01, GPIO_A1, PIN_PULLDOWN, 1);
#endif
    HAL_PIN_Set(PAD_PA06, GPIO_A6, PIN_NOPULL, 1);              //Touch screen power en

    //HAL_PIN_Set(PAD_PA09, GPIO_A9 PIN_PULLDOWN, 1);
#ifdef USB_DEBUG
    HAL_PIN_Set(PAD_PA10, DBG_DO0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA14, DBG_DO1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA31, DBG_DO2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, DBG_DO3, PIN_NOPULL, 1);
#else
    HAL_PIN_Set(PAD_PA10, I2C1_SCL, PIN_PULLUP, 1);             // I2C1(Touch)
    HAL_PIN_Set(PAD_PA14, I2C1_SDA, PIN_PULLUP, 1);
#endif


#if 0
#ifdef BSP_LCDC_USING_DBI
    HAL_PIN_Set(PAD_PA20, LCDC1_8080_WR, PIN_NOPULL, 1);        // LCDC 1 DBI8080 8bit mode
    HAL_PIN_Set(PAD_PA31, LCDC1_8080_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA34, LCDC1_8080_RD, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA36, LCDC1_8080_DC, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, LCDC1_8080_DIO0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA42, LCDC1_8080_DIO1, PIN_NOPULL, 1);

    HAL_PIN_Set(PAD_PA77, LCDC1_8080_TE, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA78, LCDC1_8080_RSTB, PIN_NOPULL, 1);
#else
    HAL_PIN_Set(PAD_PA20, LCDC1_SPI_CLK, PIN_NOPULL, 1);        // LCDC 1  QAD-SPI mode
    HAL_PIN_Set(PAD_PA31, LCDC1_SPI_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA34, LCDC1_SPI_DIO0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA36, LCDC1_SPI_DIO1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, LCDC1_SPI_DIO2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA42, LCDC1_SPI_DIO3, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA77, LCDC1_SPI_TE, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA78, LCDC1_SPI_RSTB, PIN_NOPULL, 1);
#endif /* BSP_LCDC_USING_DBI */
#endif

    HAL_PIN_Set(PAD_PA17, USART1_TXD, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA19, USART1_RXD, PIN_PULLUP, 1);


    HAL_PIN_Set(PAD_PA21, I2S2_SDO, PIN_NOPULL, 1);            // I2S2
    HAL_PIN_Set(PAD_PA23, I2S2_BCK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA25, I2S2_LRCK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA27, I2S2_SDI, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_PA41, GPIO_A41, PIN_NOPULL, 1);            // QSPI3_EN
#ifdef USB_DEBUG
    HAL_PIN_Set(PAD_PA42, DBG_DO4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA44, DBG_DO5, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA45, DBG_DO6, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA47, DBG_DO7, PIN_NOPULL, 1);
    hwp_hpsys_cfg->DBGR = HPSYS_CFG_DBGR_BITEN_L | 6;
#else
#ifdef BSP_ENABLE_QSPI3
    HAL_PIN_Set(PAD_PA44, QSPI3_CLK, PIN_NOPULL, 1);            // QSPI3
    HAL_PIN_Set(PAD_PA45, QSPI3_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA47, QSPI3_DIO0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA49, QSPI3_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA51, QSPI3_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA55, QSPI3_DIO3, PIN_PULLUP, 1);
#else
    HAL_PIN_Set(PAD_PA44, SD2_CLK, PIN_NOPULL, 1);            // SDIO2
    HAL_PIN_Set(PAD_PA45, SD2_CMD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA47, SD2_DIO0, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA49, SD2_DIO1, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA51, SD2_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA55, SD2_DIO3, PIN_PULLUP, 1);
#endif
#endif

#if 1
    HAL_PIN_Set(PAD_PA46, I2C2_SCL, PIN_PULLUP, 1);              // I2C2
    HAL_PIN_Set(PAD_PA48, I2C2_SDA, PIN_PULLUP, 1);
#else
    HAL_PIN_Set(PAD_PA46, USART1_CTS, PIN_PULLUP, 1);              // UART1 CTS
    HAL_PIN_Set(PAD_PA48, USART1_RTS, PIN_PULLUP, 1);              // UART1 RTS
#endif

    HAL_PIN_Set(PAD_PA52, SPI1_CLK, PIN_NOPULL, 1);             // SPI1
#ifdef BSP_USING_SPI_CAMERA
    HAL_PIN_Set(PAD_PA53, SPI1_CS, PIN_PULLDOWN, 1);
    //HAL_PIN_Set(PAD_PA54, SPI1_DI, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA54, SPI1_DI, PIN_PULLDOWN, 1);
#else
    HAL_PIN_Set(PAD_PA53, SPI1_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA54, SPI1_DI, PIN_PULLDOWN, 1);
#endif

    HAL_PIN_Set(PAD_PA56, SPI1_DO, PIN_NOPULL, 1);


    HAL_PIN_Set(PAD_PA58, GPIO_A58, PIN_NOPULL, 1);             // QSPI2/QSPI3 Power

    HAL_PIN_Set(PAD_PA60, QSPI2_CLK, PIN_NOPULL, 1);            // QSPI2
    HAL_PIN_Set(PAD_PA61, QSPI2_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA63, QSPI2_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA65, QSPI2_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA66, QSPI2_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA68, QSPI2_DIO3, PIN_PULLUP, 1);

    //HAL_PIN_Set(PAD_PA70, GPTIM1_CH4, PIN_NOPULL, 1);           // LCD backlight
    HAL_PIN_Set(PAD_PA77, GPIO_A77, PIN_PULLUP, 1);              //Wheel encoder
    HAL_PIN_Set(PAD_PA78, GPIO_A78, PIN_PULLUP, 1);


    HAL_PIN_Set(PAD_PA79, GPIO_A79, PIN_NOPULL, 1);             // LCD Power, LCDC_SPI_EN
#ifdef BSP_USING_SPI_CAMERA
    HAL_PIN_Set(PAD_PA80, GPTIM2_CH3, PIN_NOPULL, 1);           // Motor PWM for spi camera sensor clock
#endif
#endif


    HAL_PIN_Set(PAD_PB01, GPIO_B1, PIN_PULLUP, 0);              // I2C4 INT
    HAL_PIN_Set(PAD_PB03, GPIO_B3, PIN_NOPULL, 0);              // I2C4 power
    HAL_PIN_SetMode(PAD_PB03, 0, PIN_DIGITAL_O_NORMAL);
    HAL_PIN_Set(PAD_PB04, I2C4_SCL, PIN_PULLUP, 0);             // I2C4 (Heart rate sensor)
    HAL_PIN_Set(PAD_PB05, I2C4_SDA, PIN_PULLUP, 0);

    HAL_PIN_Set(PAD_PB06, USART5_TXD, PIN_NOPULL, 0);             // USART5
    HAL_PIN_Set(PAD_PB11, USART5_RXD, PIN_PULLUP, 0);

    HAL_PIN_Set_Analog(PAD_PB08, 0);                            // Temperature ADC
    HAL_PIN_Set_Analog(PAD_PB10, 0);                            // Battery Voltage ADC

    HAL_PIN_Set(PAD_PB12, USART4_TXD, PIN_NOPULL, 0);             // USART4
    HAL_PIN_Set(PAD_PB14, USART4_RXD, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB15, GPIO_B15,     PIN_NOPULL, 0);         //USART4_EN
    HAL_PIN_SetMode(PAD_PB15, 0, PIN_DIGITAL_O_NORMAL);

    HAL_PIN_Set(PAD_PB13, SPI3_CLK, PIN_PULLDOWN, 0);             // SPI3 (GSensor)
    HAL_PIN_Set(PAD_PB16, SPI3_DO, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB19, SPI3_DI, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB23, SPI3_CS, PIN_PULLUP, 0);

    HAL_PIN_Set(PAD_PB24, GPIO_B24, PIN_NOPULL, 0);             // SYS_EN/I2C5_INT/PMIC control
    HAL_PIN_SetMode(PAD_PB24, 0, PIN_DIGITAL_O_NORMAL);
    HAL_PIN_Set(PAD_PB25, GPIO_B25, PIN_PULLDOWN, 0);             // SPI3_EN
    //HAL_PIN_Select(PAD_PB25, 10, 0);

    HAL_PIN_Set(PAD_PB29, I2C6_SCL, PIN_PULLUP, 0);             // I2C5(PMIC control)
    HAL_PIN_Set(PAD_PB30, I2C6_SDA, PIN_PULLUP, 0);

    // PB31 SWCLK,
    // PB34 SWDIO

    HAL_PIN_Set(PAD_PB32, QSPI4_CLK, PIN_NOPULL, 0);        // QSPI4
    HAL_PIN_Set(PAD_PB33, QSPI4_CS, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB35, QSPI4_DIO0, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB36, QSPI4_DIO1, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB37, QSPI4_DIO2, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB07, QSPI4_DIO3, PIN_PULLUP, 0);

    HAL_PIN_Set(PAD_PB43, GPIO_B43, PIN_NOPULL, 0);               // SPI3_INT
    HAL_PIN_Set(PAD_PB44, GPIO_B44, PIN_PULLDOWN, 0);

    HAL_PIN_Set(PAD_PB45, USART3_TXD, PIN_NOPULL, 0);           // USART3 TX/SPI3_INT
    HAL_PIN_Set(PAD_PB46, USART3_RXD, PIN_PULLUP, 0);           // USART3 RX
    HAL_PIN_Set(PAD_PB47, GPIO_B47, PIN_PULLDOWN, 0);             // (USB_DET)Charger INT
    HAL_PIN_Set(PAD_PB48, GPIO_B48, PIN_NOPULL, 0);             // Key1

#if 0
    HAL_PIN_Set(PAD_PB32, LCDC2_SPI_CLK, PIN_NOPULL, 0);        // LCDC2  QAD-SPI mode
    HAL_PIN_Set(PAD_PB33, LCDC2_SPI_CS, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB35, LCDC2_SPI_DIO0, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB36, LCDC2_SPI_DIO1, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB37, LCDC2_SPI_DIO2, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB07, LCDC2_SPI_DIO3, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB17, GPIO_B17,       PIN_NOPULL, 0);          //LCD reset pin
    HAL_PIN_SetMode(PAD_PB17, 0, PIN_DIGITAL_O_NORMAL);
#ifdef LCD_USE_GPIO_TE
    HAL_PIN_Set(PAD_PB18, GPIO_B18, PIN_NOPULL, 0);
#else
    HAL_PIN_Set(PAD_PB18, LCDC2_SPI_TE, PIN_NOPULL, 0);
#endif /* LCD_USE_GPIO_TE */
#else
    /* doesn't use LCDC2 for now */
    HAL_PIN_Set(PAD_PB32, QSPI4_CLK, PIN_NOPULL, 0);        // QSPI4
    HAL_PIN_Set(PAD_PB33, QSPI4_CS, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB35, QSPI4_DIO0, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB36, QSPI4_DIO1, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB37, QSPI4_DIO2, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB07, QSPI4_DIO3, PIN_PULLUP, 0);

    HAL_PIN_Set(PAD_PB18, GPIO_B18, PIN_PULLUP, 0);     // QSPI4_EN

    HAL_PIN_Set(PAD_PB17, GPIO_B17, PIN_NOPULL, 0); //LCD reset pin
    HAL_PIN_SetMode(PAD_PB17, 0, PIN_DIGITAL_O_NORMAL);
#endif

}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
