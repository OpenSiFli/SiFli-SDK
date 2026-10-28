/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_board.h"

void BSP_PIN_Init(void)
{
#ifdef SOC_BF0_HCPU
    HAL_PIN_Set(PAD_PA03, GPIO_A3, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA04, GPIO_A4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA05, GPIO_A5, PIN_NOPULL, 1);
    BSP_GPIO_Set(4, 0, 1); /* Touch reset */
    BSP_GPIO_Set(5, 0, 1); /* Touch supply */

    HAL_PIN_Set(PAD_PA42, GPIO_A42, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA43, GPIO_A43, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA44, GPIO_A44, PIN_NOPULL, 1);
    BSP_GPIO_Set(42, 0, 1); /* TPS WAKEUP */
    BSP_GPIO_Set(43, 0, 1); /* TPS PWRCOM */
    BSP_GPIO_Set(44, 0, 1); /* TPS PWRUP */
    HAL_PIN_Set(PAD_PA52, GPIO_A52, PIN_PULLUP, 1);
    HAL_PIN_SetMode(PAD_PA52, 1, PIN_DIGITAL_IO_PULLUP);
    HAL_PIN_Set(PAD_PA39, GPIO_A39, PIN_NOPULL, 1);
    BSP_GPIO_Set(39, 0, 1); /* GMODE */

#ifdef BSP_USING_I2C1
    HAL_PIN_Set(PAD_PA20, I2C1_SCL, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA21, I2C1_SDA, PIN_PULLUP, 1);
#endif
#ifdef BSP_USING_I2C2
    /* Touch pull-ups are connected to the switched CTP_3.3V supply. */
    HAL_PIN_Set(PAD_PA01, I2C2_SCL, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA02, I2C2_SDA, PIN_NOPULL, 1);
#endif
#ifdef BSP_USING_SDMMC1
    HAL_PIN_Set(PAD_PA12, SD1_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA13, SD1_DIO3, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA14, SD1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA15, SD1_CMD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA16, SD1_DIO0, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA17, SD1_DIO1, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA10, GPIO_A10, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA11, GPIO_A11, PIN_PULLDOWN, 1);
    BSP_GPIO_Set(10, 0, 1);
#endif
#ifdef BSP_USING_SDMMC2
    HAL_PIN_Set(PAD_PA47, SD2_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA48, SD2_CMD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA49, SD2_DIO0, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA50, SD2_DIO1, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA45, SD2_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA46, SD2_DIO3, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA51, GPIO_A51, PIN_PULLUP, 1);
#endif
    HAL_PIN_Set(PAD_PA06, GPIO_A6, PIN_NOPULL, 1);  /* USB_DET */
    HAL_PIN_Set(PAD_PA07, GPIO_A7, PIN_PULLUP, 1);  /* CHG_STATUS */

#ifdef BSP_USING_KEY1
    HAL_PIN_Set(PAD_PA34, GPIO_A34, PIN_NOPULL, 1);
    HAL_PIN_SetMode(PAD_PA34, 1, PIN_DIGITAL_IO_NORMAL);
#endif

#if defined(BSP_USING_PDM1) && !defined(CFG_BOOTLOADER)
    HAL_PIN_Set(PAD_PA55, PDM1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA56, PDM1_DATA, PIN_PULLDOWN, 1);
#endif

    /* Set peripheral supply and reset outputs before device initialization. */
    HAL_PIN_Set(PAD_PA08, GPIO_A8, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA09, GPIO_A9, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA53, GPIO_A53, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA54, GPIO_A54, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA57, GPIO_A57, PIN_NOPULL, 1);
    BSP_GPIO_Set(9, 0, 1);  /* Wi-Fi PDN */
#if defined(BSP_USING_SDMMC1) && (SDIO_CARD_MODE == 2) && !defined(CFG_BOOTLOADER)
    /* Enable the supply before the Wi-Fi driver controls PDN on PA9. */
    BSP_GPIO_Set(8, 1, 1);  /* Wi-Fi supply */
#else
    BSP_GPIO_Set(8, 0, 1);
#endif
    BSP_GPIO_Set(53, 0, 1); /* AW99703 frontlight */
    BSP_GPIO_Set(54, 0, 1); /* Audio PA */
    BSP_GPIO_Set(57, 0, 1); /* Digital microphone supply */
#endif
}
