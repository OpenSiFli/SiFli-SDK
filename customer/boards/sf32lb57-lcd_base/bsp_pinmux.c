/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_board.h"

#if defined(BSP_USING_DCMI)
/*
 * The DVP camera shares PA32, PA33, PA43, PA44 and PA45 with the DPI/QADSPI
 * panel on this devkit, so only one of them can own those pads at a time.
 * BSP_LCD_PowerUp() re-routes them to the LCD, therefore this function is
 * also called from BSP_CAMERA_PowerUp() to take them back before capture
 * starts. Keep it free of side effects other than the pin function.
 */
void BSP_CAMERA_Pinmux(void)
{
    /* Digital Camera Interface */
    HAL_PIN_Set(PAD_PA01, DCMI_VSYNC, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA00, DCMI_HSYNC, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA26, DCMI_CLK,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA33, DCMI_DI0,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA44, DCMI_DI1,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA45, DCMI_DI2,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA43, DCMI_DI3,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA32, DCMI_DI4,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA25, DCMI_DI5,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA24, DCMI_DI6,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA06, DCMI_DI7,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA07, GPTIM2_CH1, PIN_PULLUP, 1); // mclk output
}
#endif /* BSP_USING_DCMI */

void BSP_PIN_Init(void)
{
#ifdef SOC_BF0_HCPU
    // HCPU pins

#ifndef BSP_USING_PDM1
    // Analog microphone codec ADC1 (PA55/PA56 shared with PDM, select one)
    HAL_PIN_Set_Analog(PAD_PA55, 1); // ADC1P
    HAL_PIN_Set_Analog(PAD_PA56, 1); // MICBIAS
#endif

    // SDIO1
#ifdef BSP_USING_SDMMC1
    /* single-line */
    HAL_PIN_Set(PAD_PA08, SD1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA09, SD1_CMD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA10, SD1_DIO0, PIN_PULLUP, 1);
#endif /* BSP_USING_SDMMC1 */

#ifdef BSP_USING_SDMMC2
    HAL_PIN_Set(PAD_PA02, SD2_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA03, SD2_CMD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA04, SD2_DIO0, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA05, SD2_DIO1, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA00, SD2_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA01, SD2_DIO3, PIN_PULLUP, 1);

#ifdef RT_USING_WIFI
    HAL_PIN_Set_DS0(PAD_PA00, 1, 1);
    HAL_PIN_Set_DS1(PAD_PA00, 1, 0);

    HAL_PIN_Set_DS0(PAD_PA01, 1, 1);
    HAL_PIN_Set_DS1(PAD_PA01, 1, 0);

    HAL_PIN_Set_DS0(PAD_PA02, 1, 1);
    HAL_PIN_Set_DS1(PAD_PA02, 1, 0);

    HAL_PIN_Set_DS0(PAD_PA03, 1, 1);
    HAL_PIN_Set_DS1(PAD_PA03, 1, 0);

    HAL_PIN_Set_DS0(PAD_PA04, 1, 1);
    HAL_PIN_Set_DS1(PAD_PA04, 1, 0);

    HAL_PIN_Set_DS0(PAD_PA05, 1, 1);
    HAL_PIN_Set_DS1(PAD_PA05, 1, 0);
#endif

#endif /* BSP_USING_SDMMC2 */


#if defined(BSP_USING_DCMI)
    /* Digital Camera Interface, see BSP_CAMERA_Pinmux() */
    BSP_CAMERA_Pinmux();
#endif /* BSP_USING_DCMI */

#ifdef BSP_USING_PDM1
    HAL_PIN_Set(PAD_PA56, PDM1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA55, PDM1_DATA, PIN_PULLDOWN, 1);
#endif /* BSP_USING_PDM1 */

    /* TP, AW9523 and camera share I2C1 */
    HAL_PIN_Set(PAD_PA23, I2C1_SDA, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA22, I2C1_SCL, PIN_PULLUP, 1);

#endif  /* SOC_BF0_HCPU */


}

