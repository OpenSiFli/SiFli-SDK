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
    uint32_t pid = (hwp_hpsys_cfg->IDR & HPSYS_CFG_IDR_PID) >> HPSYS_CFG_IDR_PID_Pos;

    if (pid == 0x1)
    {
        HAL_PIN_Set(PAD_SIP01, PSRAM_DQ0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SIP02, PSRAM_DQ1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SIP03, PSRAM_DQ2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SIP04, PSRAM_DQ3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SIP05, PSRAM_CS, PIN_NOPULL, 1);

        HAL_PIN_Set(PAD_SIP07, PSRAM_CLK, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SIP08, PSRAM_DQ4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SIP09, PSRAM_DQ5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SIP10, PSRAM_DQ6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SIP11, PSRAM_DQ7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SIP12, PSRAM_DQS0, PIN_PULLDOWN, 1);
    }
#ifdef BSP_USING_EXT_PSRAM
    HAL_PIN_Set(PAD_PA02, PSRAM_DQ0, PIN_PULLDOWN, 1);           // OPI PSRAM External, XCEELA interface, 16MB on board.
    HAL_PIN_Set(PAD_PA04, PSRAM_DQ1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA05, PSRAM_DQ2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA06, PSRAM_DQ3, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA09, PSRAM_DQ4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA11, PSRAM_DQ5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA12, PSRAM_DQ6, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA13, PSRAM_DQ7, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA15, PSRAM_DQS0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA20, PSRAM_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA07, PSRAM_CS, PIN_NOPULL, 1);

#ifdef BSP_USING_DUAL_PSRAM
    HAL_PIN_Set(PAD_PA18, PSRAM_DQ8, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA22, PSRAM_DQ9, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA24, PSRAM_DQ10, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA32, PSRAM_DQ11, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA33, PSRAM_DQ12, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA59, PSRAM_DQ13, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA62, PSRAM_DQ14, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA64, PSRAM_DQ15, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA26, PSRAM_DQS1, PIN_PULLDOWN, 1);
#endif /* BSP_USING_DUAL_PSRAM */

    HAL_PIN_Set(PAD_PA67, GPIO_A67, PIN_PULLUP, 1);     // PSRAM EN
#endif  //BSP_USING_EXT_PSRAM

    HAL_PIN_Set(PAD_PA00, GPIO_A0, PIN_NOPULL, 1);              //Touch screen / i2c1 reset
    HAL_PIN_Set(PAD_PA01, GPIO_A1, PIN_PULLDOWN, 1);             //SD1_EN/USB_DP/I2S1_BCK/PDM1_CLK
    HAL_PIN_Set(PAD_PA03, GPIO_A3, PIN_NOPULL, 1);              //Touch screen / i2c1  power en

    HAL_PIN_Set(PAD_PA08, I2S1_LRCK, PIN_NOPULL, 1);     // I2S1
    HAL_PIN_Set(PAD_PA35, I2S1_BCK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA70, I2S1_SDI, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_PA10, I2C1_SCL, PIN_PULLUP, 1);             // I2C1(Touch)
    HAL_PIN_Set(PAD_PA14, I2C1_SDA, PIN_PULLUP, 1);

    HAL_PIN_Set(PAD_PA17, USART1_TXD, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA19, USART1_RXD, PIN_PULLUP, 1);


    HAL_PIN_Set(PAD_PA21, I2S2_SDO, PIN_NOPULL, 1);            // I2S2
    HAL_PIN_Set(PAD_PA23, I2S2_BCK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA25, I2S2_LRCK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA27, I2S2_SDI, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_PA28, SD1_DIO0, PIN_PULLUP, 1);       // SDIO1
    HAL_PIN_Set(PAD_PA29, SD1_DIO1, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA30, SD1_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA31, SD1_DIO3, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA47, SD1_DIO4, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA49, SD1_DIO5, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA51, SD1_DIO6, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA55, SD1_DIO7, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA34, SD1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA36, SD1_CMD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA37, GPIO_A37, PIN_PULLUP, 1);     // SDIO1 RESET

    HAL_PIN_Set(PAD_PA38, GPIO_A38, PIN_NOPULL, 1);            // UART1_EN

    HAL_PIN_Set(PAD_PA43, GPIO_A43, PIN_NOPULL, 1);             // NFC_VEN

    HAL_PIN_Set(PAD_PA46, I2C2_SCL, PIN_PULLUP, 1);              // I2C2
    HAL_PIN_Set(PAD_PA48, I2C2_SDA, PIN_PULLUP, 1);

    HAL_PIN_Set(PAD_PA52, SPI1_CLK, PIN_NOPULL, 1);             // SPI1
    HAL_PIN_Set(PAD_PA53, SPI1_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA54, SPI1_DI, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA56, SPI1_DO, PIN_NOPULL, 1);


    HAL_PIN_Set(PAD_PA58, GPIO_A58, PIN_NOPULL, 1);             // QSPI2 Power

    HAL_PIN_Set(PAD_PA60, QSPI2_CLK, PIN_NOPULL, 1);            // QSPI2
    HAL_PIN_Set(PAD_PA61, QSPI2_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA63, QSPI2_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA65, QSPI2_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA66, QSPI2_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA68, QSPI2_DIO3, PIN_PULLUP, 1);

    HAL_PIN_Set(PAD_PA77, GPIO_A77, PIN_PULLUP, 1);              //NFC_INT
    //HAL_PIN_Set(PAD_PA78, GPIO_A78, PIN_PULLUP, 1);


    HAL_PIN_Set(PAD_PA79, GPIO_A79, PIN_NOPULL, 1);             // I2C1_INT

    HAL_PIN_Set(PAD_PA80, GPIO_A80, PIN_NOPULL, 1);           // USART1_INT


#ifdef BSP_LCDC_USING_DPI
    HAL_PIN_Set(PAD_PA17, LCDC1_DPI_CLK,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA18, LCDC1_DPI_DE,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA19, LCDC1_DPI_R0,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA21, LCDC1_DPI_R1,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA22, LCDC1_DPI_R2,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA23, LCDC1_DPI_R3,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA24, LCDC1_DPI_R4,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA25, LCDC1_DPI_R5,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA26, LCDC1_DPI_R6,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA27, LCDC1_DPI_R7,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA28, LCDC1_DPI_G0,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA29, LCDC1_DPI_G1,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA30, LCDC1_DPI_G2,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA31, LCDC1_DPI_G3,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA32, LCDC1_DPI_G4,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA33, LCDC1_DPI_G5,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA34, LCDC1_DPI_G6,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA35, LCDC1_DPI_G7,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA36, LCDC1_DPI_B0,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, LCDC1_DPI_B1,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA41, LCDC1_DPI_B2,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA42, LCDC1_DPI_B3,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA43, LCDC1_DPI_B4,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA44, LCDC1_DPI_B5,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA45, LCDC1_DPI_B6,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA46, LCDC1_DPI_B7,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA59, LCDC1_DPI_HSYNC,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA62, LCDC1_DPI_VSYNC,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA64, LCDC1_DPI_SD,  PIN_NOPULL, 1);
    //HAL_PIN_Set(PAD_PA67, LCDC1_DPI_CM,  PIN_NOPULL, 1);
#endif /* BSP_LCDC_USING_DPI */

#endif /*SOC_BF0_HCPU*/






    HAL_PIN_Set(PAD_PB00, QSPI4_CLK, PIN_NOPULL, 0);        // QSPI4
    HAL_PIN_Set(PAD_PB01, QSPI4_CS, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB02, QSPI4_DIO0, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB03, QSPI4_DIO1, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB21, QSPI4_DIO2, PIN_PULLUP, 0);
#if 1 // for sip psram, set dio 3 to this pin
    HAL_PIN_Set(PAD_PB07, QSPI4_DIO3, PIN_PULLUP, 0);
#else   // out ext flash, may need use this pin
    HAL_PIN_Set(PAD_PB41, QSPI4_DIO3, PIN_PULLUP, 0);
#endif
    HAL_PIN_Set(PAD_PB20, GPIO_B20, PIN_PULLUP, 0);         //QSPI4_EN

    HAL_PIN_Set(PAD_PB04, I2C4_SCL, PIN_PULLUP, 0);             // I2C4 (Heart rate sensor)
    HAL_PIN_Set(PAD_PB05, I2C4_SDA, PIN_PULLUP, 0);

    HAL_PIN_Set(PAD_PB44, GPIO_B44, PIN_PULLUP, 0);              // I2C4 INT
    HAL_PIN_Set(PAD_PB09, GPIO_B9, PIN_NOPULL, 0);              // I2C4 power
    HAL_PIN_SetMode(PAD_PB09, 0, PIN_DIGITAL_O_NORMAL);

    HAL_PIN_Set(PAD_PB06, USART5_TXD, PIN_NOPULL, 0);             // USART5
    HAL_PIN_Set(PAD_PB11, USART5_RXD, PIN_PULLUP, 0);

    HAL_PIN_Set(PAD_PB32, LCDC2_SPI_CLK, PIN_NOPULL, 0);        // LCDC2  QAD-SPI mode
    HAL_PIN_Set(PAD_PB33, LCDC2_SPI_CS, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB35, LCDC2_SPI_DIO0, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB36, LCDC2_SPI_DIO1, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB37, LCDC2_SPI_DIO2, PIN_NOPULL, 0);
    //HAL_PIN_Set(PAD_PB07, LCDC2_SPI_DIO3, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB18, LCDC2_SPI_TE, PIN_NOPULL, 0);

#ifdef SOC_BF0_LCPU
    HAL_PIN_Set(PAD_PB17, GPIO_B17, PIN_PULLUP, 0);             //LCD reset pin
    HAL_PIN_SetMode(PAD_PB17, 0, PIN_DIGITAL_O_NORMAL);
#endif

    HAL_PIN_Set(PAD_PB38, GPIO_B38, PIN_NOPULL, 0);             // LCDC2_SPI_EN

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

    HAL_PIN_Set(PAD_PB25, GPIO_B25, PIN_PULLUP, 0);             // SPI3_EN
    HAL_PIN_Set(PAD_PB43, GPIO_B43, PIN_NOPULL, 0);               // SPI3_INT

    HAL_PIN_Set(PAD_PB24, GPIO_B24, PIN_NOPULL, 0);             // AUTORESET_PWM
    HAL_PIN_SetMode(PAD_PB24, 0, PIN_DIGITAL_O_NORMAL);

    HAL_PIN_Set(PAD_PB26, GPIO_B26, PIN_NOPULL, 0);               // CHARGE_STA_DET
    //HAL_PIN_Select(PAD_PB26, 10, 0);
    HAL_PIN_Set(PAD_PB27, GPIO_B27, PIN_NOPULL, 0);           // SYS_EN
    HAL_PIN_Set(PAD_PB28, GPIO_B28, PIN_NOPULL, 0);           // CHARGE_EN

    HAL_PIN_Set(PAD_PB29, I2C6_SCL, PIN_PULLUP, 0);             // I2C6(PMIC control)
    HAL_PIN_Set(PAD_PB30, I2C6_SDA, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB48, GPIO_B48, PIN_NOPULL, 0);             // I2C6_INT
    // PB31 SWCLK,
    // PB34 SWDIO
    HAL_PIN_Set(PAD_PB22, GPIO_B22, PIN_NOPULL, 0);         // I2C5_INT
    HAL_PIN_Set(PAD_PB39, I2C5_SCL, PIN_PULLUP, 0);             // I2C5
    HAL_PIN_Set(PAD_PB40, I2C5_SDA, PIN_PULLUP, 0);

    HAL_PIN_Set(PAD_PB45, USART3_TXD, PIN_NOPULL, 0);           // USART3 TX
    HAL_PIN_Set(PAD_PB46, USART3_RXD, PIN_PULLUP, 0);           // USART3 RX

    HAL_PIN_Set(PAD_PB47, GPIO_B47, PIN_PULLDOWN, 0);             // (USB_DET)Charger INT

}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
