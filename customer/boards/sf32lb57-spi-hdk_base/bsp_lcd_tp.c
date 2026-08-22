/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_board.h"

#define LCD_RESET_PIN           (0)            // GPIO_A0
#define TP_RESET                (10)            // GPIO_A10


extern void BSP_GPIO_Set(int pin, int val, int is_porta);

void BSP_LCD_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(LCD_RESET_PIN, high1_low0, 1);
}


void BSP_LCD_PowerDown(void)
{
    BSP_LCD_Reset(0);
}

static void QSPI_PIN_Set(int pad, pin_function func, int flags, int hcpu)
{
    HAL_PIN_Set(pad, func, flags, hcpu);
#ifdef BSP_LCDC_USING_DDR_QADSPI
    // Set the pin driver strength to 12mA
    HAL_PIN_Set_DS0(pad, 1, 1);
    HAL_PIN_Set_DS1(pad, 1, 1);
#else
    // Set the pin driver strength to 8mA
    HAL_PIN_Set_DS0(pad, 1, 1);
    HAL_PIN_Set_DS1(pad, 1, 0);
#endif /* BSP_LCDC_USING_DDR_QADSPI */
}
void BSP_LCD_PowerUp(void)
{
#ifdef BSP_LCDC_USING_QADSPI

    QSPI_PIN_Set(PAD_PA04, LCDC1_SPI_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA03, LCDC1_SPI_CS, PIN_NOPULL, 1);
    QSPI_PIN_Set(PAD_PA05, LCDC1_SPI_DIO0, PIN_NOPULL, 1);
    QSPI_PIN_Set(PAD_PA06, LCDC1_SPI_DIO1, PIN_NOPULL, 1);
    QSPI_PIN_Set(PAD_PA07, LCDC1_SPI_DIO2, PIN_NOPULL, 1);
    QSPI_PIN_Set(PAD_PA08, LCDC1_SPI_DIO3, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA02, LCDC1_SPI_TE, PIN_NOPULL, 1);

    HAL_PIN_Set(PAD_PA01, GPTIM1_CH4, PIN_NOPULL, 1);   // LCDC1_BL_PWM_CTRL, LCD backlight PWM
#endif /* BSP_LCDC_USING_QADSPI */

    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_LCDC, RCC_CLK_LCDC_SYSCLK);

}

void BSP_TP_PowerUp(void)
{
    HAL_PIN_Set(PAD_PA00 + TP_RESET, GPIO_A0 + TP_RESET, PIN_NOPULL, 1);
    BSP_GPIO_Set(TP_RESET,  1, 1);
}

void BSP_TP_PowerDown(void)
{
    BSP_GPIO_Set(TP_RESET,  0, 1);
}

void BSP_TP_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(TP_RESET, high1_low0, 1);
}

