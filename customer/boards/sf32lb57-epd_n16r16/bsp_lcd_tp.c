/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_board.h"
#ifdef BSP_USING_RTTHREAD
#include <rtthread.h>
#endif

void BSP_LCD_Reset(uint8_t level)
{
    /* The EPD panel has no reset pin. */
    (void)level;
}

void BSP_LCD_PowerDown(void)
{
    BSP_GPIO_Set(43, 0, 1);
    BSP_GPIO_Set(44, 0, 1);
    BSP_GPIO_Set(39, 0, 1); /* GMODE */
}

static void epd_pin_set(int pad, pin_function function)
{
    GPIO_InitTypeDef input = {.Pin = pad - PAD_PA00, .Mode = GPIO_MODE_INPUT, .Pull = GPIO_NOPULL};
    HAL_GPIO_Init(hwp_gpio1, &input);
    HAL_PIN_SetMode(pad, 1, PIN_DIGITAL_IO_NORMAL);
    int result = HAL_PIN_Set(pad, function, PIN_NOPULL, 1);
    HAL_ASSERT(result == 0);
    (void)result;
}

void BSP_LCD_PowerUp(void)
{
    epd_pin_set(PAD_PA24, LCDC1_TCON_SDCLK);
    epd_pin_set(PAD_PA25, LCDC1_TCON_SDLE);
    epd_pin_set(PAD_PA26, LCDC1_TCON_SDOE);
    epd_pin_set(PAD_PA27, LCDC1_TCON_SDSTL);
    epd_pin_set(PAD_PA28, LCDC1_TCON_SDAT0);
    epd_pin_set(PAD_PA29, LCDC1_TCON_SDAT1);
    epd_pin_set(PAD_PA30, LCDC1_TCON_SDAT2);
    epd_pin_set(PAD_PA31, LCDC1_TCON_SDAT3);
    epd_pin_set(PAD_PA32, LCDC1_TCON_SDAT4);
    epd_pin_set(PAD_PA33, LCDC1_TCON_SDAT5);
    epd_pin_set(PAD_PA37, LCDC1_TCON_SDAT6);
    epd_pin_set(PAD_PA38, LCDC1_TCON_SDAT7);
    epd_pin_set(PAD_PA40, LCDC1_TCON_GDCLK);
    epd_pin_set(PAD_PA41, LCDC1_TCON_GDSP);

    HAL_PIN_Set(PAD_PA39, GPIO_A39, PIN_NOPULL, 1);
    BSP_GPIO_Set(39, 0, 1);
    HAL_PIN_Set(PAD_PA42, GPIO_A42, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA43, GPIO_A43, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA44, GPIO_A44, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA52, GPIO_A52, PIN_PULLUP, 1);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_LCDC, RCC_CLK_LCDC_SYSCLK);
}

void BSP_LCD_GMODE_Set(uint8_t level)
{
    BSP_GPIO_Set(39, level, 1);
}

void BSP_TP_Reset(uint8_t level)
{
    HAL_PIN_Set(PAD_PA04, GPIO_A4, PIN_NOPULL, 1);
    BSP_GPIO_Set(4, level, 1);
}

void BSP_TP_PowerUp(void)
{
    HAL_PIN_Set(PAD_PA05, GPIO_A5, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA03, GPIO_A3, PIN_NOPULL, 1);
    BSP_TP_Reset(0);
    HAL_Delay_us(100);
    BSP_GPIO_Set(5, 1, 1);
    HAL_Delay_us(1000);
    BSP_TP_Reset(1);
#ifdef BSP_USING_RTTHREAD
    rt_thread_mdelay(200);
#else
    HAL_Delay(200);
#endif
}

void BSP_TP_PowerDown(void)
{
    BSP_TP_Reset(0);
    HAL_Delay_us(100);
    BSP_GPIO_Set(5, 0, 1);
#ifdef BSP_USING_RTTHREAD
    rt_thread_mdelay(5);
#else
    HAL_Delay(5);
#endif
}
