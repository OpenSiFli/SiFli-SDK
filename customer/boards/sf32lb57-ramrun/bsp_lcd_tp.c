/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_board.h"

#define LCD_RESET_PIN           (0)         // GPIO_A00
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

void BSP_LCD_PowerUp(void)
{

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

