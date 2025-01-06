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
    if (HAL_SIP_AUTO_CFG() != 0) // no sip mem get, use configure or default
    {
#ifdef PSRAM_BL_MODE
        if (PSRAM_BL_MODE == 6)
        {
            HAL_PIN_SetFlash1_WithMode(SFPIN_SIP1_WINB_HYP64);
        }
        else
#endif
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

#if (BSP_QSPI1_MODE == 5)    //SPI_MODE_LEGPSRAM
            HAL_PIN_Set(PAD_SA05, MPI1_CS,   PIN_NOPULL, 1);
            HAL_PIN_Set(PAD_SA00, MPI1_DM, PIN_PULLDOWN, 1);
            HAL_PIN_Set(PAD_SA06, MPI1_CLKB, PIN_NOPULL, 1);
            HAL_PIN_Set(PAD_SA12, MPI1_DQS, PIN_PULLDOWN, 1);
#else
            HAL_PIN_Set(PAD_SA06, MPI1_CS,   PIN_NOPULL, 1);
            HAL_PIN_Set(PAD_SA12, MPI1_DQSDM, PIN_PULLDOWN, 1);
            HAL_PIN_Set_Analog(PAD_SA00, 1);
            HAL_PIN_Set_Analog(PAD_SA05, 1);
#endif /* BSP_QSPI1_MODE */
        }
    }
#else
    // MPI2 PSRAM
    if (HAL_SIP_AUTO_CFG() != 0) // no sip mem get, use configure or default
    {
#if (BSP_QSPI2_MODE == 5)    //SPI_MODE_LEGPSRAM, 565
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

#else
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
        HAL_PIN_Set_Analog(PAD_SB00, 1);
        HAL_PIN_Set_Analog(PAD_SB10, 1);
#endif
    }
#endif /* BSP_ENABLE_MPI1 */

#endif /* BSP_USING_PSRAM */

    HAL_PIN_Set(PAD_PA05, GPIO_A5, PIN_NOPULL, 1);// LCDC1_QSPI_RSTB

#ifdef BSP_ENABLE_MPI3
    // MPI3
    HAL_PIN_Set(PAD_PA06, MPI3_CS,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA07, MPI3_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA08, MPI3_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA09, MPI3_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA10, MPI3_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA11, MPI3_DIO3, PIN_PULLUP, 1);
#endif

#ifdef BSP_USING_USBD
    HAL_PIN_Set_Analog(PAD_PA17, 1);
    HAL_PIN_Set_Analog(PAD_PA18, 1);
#else
    // UART1
    HAL_PIN_Set(PAD_PA17, USART1_TXD, PIN_NOPULL, 1);   // KEY1/UART1_TXD
    HAL_PIN_Set(PAD_PA18, USART1_RXD, PIN_PULLUP, 1);   // KEY2/UART1_RXD
#endif

    HAL_PIN_Set(PAD_PA28, GPIO_A28, PIN_NOPULL, 1);     // AUDIO_PA_CTRL
    HAL_PIN_Set(PAD_PA31, GPIO_A31, PIN_NOPULL, 1);     //Motor_PWM_Ctrl

    // LCDC1

    HAL_PIN_Set(PAD_PA33, LCDC1_SPI_TE, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA36, LCDC1_SPI_CS, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA37, LCDC1_SPI_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, LCDC1_SPI_DIO0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA39, LCDC1_SPI_DIO1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA40, LCDC1_SPI_DIO2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA41, LCDC1_SPI_DIO3, PIN_NOPULL, 1);


#ifdef BSP_LCDC_USING_DDR_QADSPI
    if (__HAL_SYSCFG_GET_REVID() < 2)
    {
        HAL_PIN_Set_DS0(PAD_PA37, 1, 1);
        HAL_PIN_Set_DS0(PAD_PA38, 1, 0);
        HAL_PIN_Set_DS0(PAD_PA39, 1, 0);
        HAL_PIN_Set_DS0(PAD_PA40, 1, 0);
        HAL_PIN_Set_DS0(PAD_PA41, 1, 0);

        HAL_PIN_Set_DS1(PAD_PA37, 1, 1);
        HAL_PIN_Set_DS1(PAD_PA38, 1, 0);
        HAL_PIN_Set_DS1(PAD_PA39, 1, 0);
        HAL_PIN_Set_DS1(PAD_PA40, 1, 0);
        HAL_PIN_Set_DS1(PAD_PA41, 1, 0);
    }
#endif /* DDR_LCD */


    // I2C1 for touch panel
    HAL_PIN_Set(PAD_PA48, I2C1_SCL, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA49, I2C1_SDA, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA50, GPIO_A50, PIN_NOPULL, 1);     // CTP_WKUP_INT

    HAL_PIN_Set(PAD_PA51, GPIO_A51, PIN_PULLDOWN, 1);   // CHARG IN DETECT
    // PAD_PA55   XTAL32K_XI
    // PAD_PA56   XTAL32K_XO

#if defined(LCD_USING_PWM_AS_BACKLIGHT)
    HAL_PIN_Set(PAD_PB26, GPTIM3_CH4, PIN_NOPULL, 0);   // LCDC1_BL_PWM_CTRL, LCD backlight PWM
#else
    HAL_PIN_Set(PAD_PB26, GPIO_B26, PIN_NOPULL, 0);     // LCDC1_BL_PWM_CTRL, LCD backlight PWM
#endif

#endif

#if defined(BSP_USING_BOARD_EH_SS6700XXX)
    HAL_PIN_Set(PAD_PB04, LCDC1_SPI_RSTB, PIN_PULLDOWN, 0);// LCDC1_QSPI_RSTB, LCD reset pin
    HAL_PIN_Set(PAD_PB05, GPIO_B5, PIN_PULLDOWN, 0);// GPS_RTC_IN, GPS wakeup pin
#endif
    // PAD_B13 SWDIO
    // PAD_B15 SWDCLK
    // UART4
    HAL_PIN_Set(PAD_PB16, USART4_RXD, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB17, USART4_TXD, PIN_PULLUP, 0);

    HAL_PIN_Set(PAD_PB18, GPIO_B18, PIN_NOPULL, 0);  // CHARG

    // GPS_UART - UART5
    HAL_PIN_Set(PAD_PB19, USART5_RXD, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB20, USART5_TXD, PIN_PULLUP, 0);

    HAL_PIN_Set(PAD_PB21, GPIO_B21, PIN_PULLDOWN, 0);  // GS_HR_GPS_EN

    HAL_PIN_Set(PAD_PB22, GPIO_B22, PIN_NOPULL, 0);  // TWI_CLK / HR_RESET/HR_VCC_EN
    HAL_PIN_Set(PAD_PB23, GPIO_B23, PIN_NOPULL, 0);  //TWI_SDA  /GS_HR_SCL
    HAL_PIN_Set(PAD_PB24, I2C5_SDA, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB25, GPIO_B25, PIN_NOPULL, 0);  // CTP_RESET, touch panel reset

    // TODO: Check analog pin config.                 // #GPADC_CH4, NTC_GPADC_CH4, Battery temperature

    HAL_PIN_Set_Analog(PAD_PB27, 0);                  // #GPADC_CH5, VBAT_GPADC_CH5, Battery voltage

    HAL_PIN_Set(PAD_PB32, GPIO_B32, PIN_NOPULL, 0);  // KEY_LONGPRESS_RESET, Function key + long press reset
    HAL_PIN_Set(PAD_PB33, GPIO_B33, PIN_NOPULL, 0);  // GS_WKUP_INT, Acc sensor interrupt
    HAL_PIN_Set(PAD_PB34, GPIO_B34, PIN_NOPULL, 0);  // HR_WKUP_INT, Heartrate sensor interrupt


#ifdef SOC_BF0_HCPU

    if (PM_STANDBY_BOOT != SystemPowerOnModeGet())

    {
        // TODO: set following pin
        // PAD_PBR0 SIP_VCC_EN/32K_CLK_OUTPUT,
        // PAD_PBR1 Charger INT, Charger plug interrupt
        // PAD_PBR2 Charger INT, Charger plug interrupt
        /* Force PBR0 always output 1 for PWR_REQ function */
        HAL_PBR0_FORCE1_ENABLE();
        HAL_PBR_ConfigMode(1, 1);       //PBR1 1 outmode: GPS WAKE MCU
        HAL_PBR_ConfigMode(2, 1);       //PBR2 1 outmode: PBR2_LCD_EN
    }
#endif
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
