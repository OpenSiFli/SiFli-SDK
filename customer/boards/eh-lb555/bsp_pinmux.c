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

#include "string.h"
#include "rtconfig.h"

#ifdef BSP_USING_RTTHREAD
    #include "board.h"
    #include "drv_common.h"
    #include "drv_psram.h"
    #include "drv_io.h"
    #include "log.h"
#endif

//#define USB_DEBUG

#include "bf0_hal.h"

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


#ifdef BSP_USING_USBD
    HAL_PIN_Set_Analog(PAD_PA01, 1);
    HAL_PIN_Set_Analog(PAD_PA03, 1);
#elif defined(BSP_USING_PDM1)
    HAL_PIN_Set(PAD_PA01, PDM1_CLK, PIN_PULLUP, 1);                 // PDM 1
    HAL_PIN_Set(PAD_PA03, PDM1_DATA, PIN_PULLDOWN, 1);
#endif

    HAL_PIN_Set(PAD_PA05, USART2_RXD, PIN_PULLUP, 1);           // USART2 RX
    HAL_PIN_Set(PAD_PA07, USART2_TXD, PIN_NOPULL, 1);           // USART2 TX

    HAL_PIN_Set(PAD_PA06, GPIO_A6, PIN_PULLDOWN, 1);            //VDD18_DVDD_EN

    HAL_PIN_Set(PAD_PA08, GPIO_A8, PIN_PULLUP, 1);              //I2SDAC_RST
    HAL_PIN_Set(PAD_PA09, GPIO_A9, PIN_PULLUP, 1);              //I2SDAC_INT

    //HAL_PIN_Set(PAD_PA09, GPIO_A9 PIN_PULLDOWN, 1);
#ifdef USB_DEBUG
    HAL_PIN_Set(PAD_PA10, DBG_DO0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA14, DBG_DO1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA31, DBG_DO2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, DBG_DO3, PIN_NOPULL, 1);
#else
    HAL_PIN_Set(PAD_PA10, I2C1_SCL, PIN_PULLUP, 1);             // I2C1(Touch)/I2SDAC
    HAL_PIN_Set(PAD_PA14, I2C1_SDA, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA53, GPIO_A53, PIN_PULLUP, 1);             // CTP_RST
    HAL_PIN_Set(PAD_PA79, GPIO_A79, PIN_PULLUP, 1);             // CTP INT
#endif

    HAL_PIN_Set(PAD_PA17, USART1_TXD, PIN_NOPULL, 1);          // UART1
    HAL_PIN_Set(PAD_PA19, USART1_RXD, PIN_PULLUP, 1);

    HAL_PIN_Set(PAD_PA21, I2S2_SDO, PIN_NOPULL, 1);            // I2S2
    HAL_PIN_Set(PAD_PA23, I2S2_BCK, PIN_NOPULL, 1);            // I2S2_BCK(PDM2_CLK)
    HAL_PIN_Set(PAD_PA44, I2S2_BCK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA25, I2S2_LRCK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA27, I2S2_SDI, PIN_PULLDOWN, 1);          // I2S2_SDI(PDM2_DATA)

    HAL_PIN_Set(PAD_PA41, GPIO_A41, PIN_PULLDOWN, 1);            // BLE_LEVELSHIFT_EN

#ifdef USB_DEBUG
    HAL_PIN_Set(PAD_PA42, DBG_DO4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA44, DBG_DO5, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA45, DBG_DO6, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA47, DBG_DO7, PIN_NOPULL, 1);
    hwp_hpsys_cfg->DBGR = HPSYS_CFG_DBGR_BITEN_L | 6;
#else
#ifdef BSP_ENABLE_QSPI3
    HAL_PIN_Set(PAD_PA00, QSPI3_CLK, PIN_PULLDOWN, 1);          //QSPI3
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

    HAL_PIN_Set(PAD_PA50, I2C3_SCL, PIN_PULLUP, 1);             // I2C3
    HAL_PIN_Set(PAD_PA57, I2C3_SDA, PIN_PULLUP, 1);

    HAL_PIN_Set(PAD_PA52, GPIO_A52, PIN_PULLUP, 1);             // BT_PWREN


    HAL_PIN_Set(PAD_PA54, GPIO_A54, PIN_PULLDOWN, 1);           // I2SDAC_VBAT_EN
    HAL_PIN_Set(PAD_PA56, GPTIM2_CH4, PIN_PULLDOWN, 1);         // MOT_PWM

    HAL_PIN_Set(PAD_PA58, GPIO_A58, PIN_NOPULL, 1);             // QSPI2_EN
    HAL_PIN_Set(PAD_PA60, QSPI2_CLK, PIN_NOPULL, 1);            // QSPI2
    HAL_PIN_Set(PAD_PA61, QSPI2_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA63, QSPI2_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA65, QSPI2_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA66, QSPI2_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA68, QSPI2_DIO3, PIN_PULLUP, 1);

    HAL_PIN_Set(PAD_PA77, GPIO_A77, PIN_PULLUP, 1);              //GNSS_WAKEUP_MCU
    HAL_PIN_Set(PAD_PA78, GPIO_A78, PIN_PULLUP, 1);              //FUN_KEY
    HAL_PIN_Set(PAD_PA79, GPIO_A79, PIN_PULLUP, 1);              //TP INT

    HAL_PIN_Set(PAD_PA80, GPIO_A80, PIN_PULLUP, 1);             // BT_WAKEUP_MCU
#endif

    HAL_PIN_Set(PAD_PB01, GPIO_B1, PIN_PULLUP, 0);              // BT_PWREN
    HAL_PIN_Set(PAD_PB03, GPIO_B3, PIN_PULLDOWN, 0);            // SPK_PA_EN

    HAL_PIN_Set(PAD_PB04, I2C4_SCL, PIN_PULLUP, 0);             // I2C4 (Heart rate sensor)
    HAL_PIN_Set(PAD_PB05, I2C4_SDA, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB06, GPIO_B6, PIN_PULLUP, 0);              // HRD_RSTN
    HAL_PIN_Set(PAD_PB44, GPIO_B44, PIN_PULLUP, 0);             // HRD_INT

    HAL_PIN_Set(PAD_PB07, GPIO_B7, PIN_PULLUP, 0);              // QSPI_DB1
    HAL_PIN_Set(PAD_PB09, GPIO_B9, PIN_PULLUP, 0);              // RESET_FR5081

    HAL_PIN_Set_Analog(PAD_PB08, 0);                            // Temperature ADC
    HAL_PIN_Set_Analog(PAD_PB10, 0);                            // Battery Voltage ADC
    HAL_PIN_Set_Analog(PAD_PB13, 0);                            // TEMP_ADC

    HAL_PIN_Set(PAD_PB12, USART4_TXD, PIN_NOPULL, 0);           // USART4(GPS)
    HAL_PIN_Set(PAD_PB14, USART4_RXD, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB15, GPIO_B15,     PIN_NOPULL, 0);         // GNSS_CHIP_EN(USART4_EN)
    HAL_PIN_SetMode(PAD_PB15, 0, PIN_DIGITAL_O_NORMAL);
    HAL_PIN_Set(PAD_PB23, GPIO_B23, PIN_PULLUP, 0);             // MCU_WAKEUP_GNSS

    HAL_PIN_Set(PAD_PB11, GPIO_B11, PIN_NOPULL, 0);              // LCD_VCI_EN
    HAL_PIN_Set(PAD_PB17, GPIO_B17, PIN_NOPULL, 0);         // LCD reset pin
    HAL_PIN_Set(PAD_PB18, GPIO_B18, PIN_PULLUP, 0);         // LCD_TE
    HAL_PIN_Set(PAD_PB32, QSPI4_CLK, PIN_NOPULL, 0);        // QSPI4
    HAL_PIN_Set(PAD_PB33, QSPI4_CS, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB35, QSPI4_DIO0, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB36, QSPI4_DIO1, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB37, QSPI4_DIO2, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB07, QSPI4_DIO3, PIN_PULLUP, 0);

    HAL_PIN_Set(PAD_PB19, GPIO_B19, PIN_PULLDOWN, 0);       // GPIO_ENCODE_A
    HAL_PIN_Set(PAD_PB22, GPIO_B22, PIN_PULLUP, 0);         // GPIO_ENCODE_B

    HAL_PIN_Set(PAD_PB24, GPIO_B24, PIN_PULLDOWN, 0);       // TWI_SDA
    HAL_PIN_Set(PAD_PB25, GPIO_B25, PIN_PULLDOWN, 0);       // TWI_SCL
    HAL_PIN_Set(PAD_PB26, GPIO_B26, PIN_PULLUP, 0);         // CHRG
    HAL_PIN_Set(PAD_PB28, GPIO_B28, PIN_PULLUP, 0);         // STDBY

    HAL_PIN_Set(PAD_PB27, GPIO_B27, PIN_PULLUP, 0);         // RTC_EINT

    HAL_PIN_Set(PAD_PB29, I2C6_SCL, PIN_PULLUP, 0);         // I2C6(GSensor)
    HAL_PIN_Set(PAD_PB30, I2C6_SDA, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB43, GPIO_B43, PIN_NOPULL, 0);         // AG_INT0

    //HAL_PIN_Set(PAD_PB31, GPIO_B31, PIN_NOPULL, 0);       // SWCLK
    //HAL_PIN_Set(PAD_PB34, GPIO_B34, PIN_NOPULL, 0);       // SWDIO



    HAL_PIN_Set(PAD_PB45, USART3_TXD, PIN_NOPULL, 0);       // USART3 TX/SPI3_INT
    HAL_PIN_Set(PAD_PB46, USART3_RXD, PIN_PULLUP, 0);       // USART3 RX

    HAL_PIN_Set(PAD_PB47, GPIO_B47, PIN_PULLUP, 0);         // (USB_DET)Charger INT
    HAL_PIN_Set(PAD_PB48, GPIO_B48, PIN_PULLUP, 0);         // PWRKEY_MCU
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
