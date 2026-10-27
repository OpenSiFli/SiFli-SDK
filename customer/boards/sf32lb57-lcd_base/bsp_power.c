/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_board.h"
#include "aw9523.h"

void aw9523_ensure_init(void)
{
    static uint8_t inited = 0;
    if (!inited && aw9523_init() == 0)
        inited = 1;
}

void BSP_GPIO_Set(int pin, int val, int is_porta)
{
    GPIO_TypeDef *gpio = hwp_gpio1;
    GPIO_InitTypeDef GPIO_InitStruct;

    // set sensor pin to output mode
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &GPIO_InitStruct);

    // set sensor pin to high == power on sensor board
    HAL_GPIO_WritePin(gpio, pin, (GPIO_PinState)val);
}

__WEAK void BSP_PowerDownCustom(int coreid, bool is_deep_sleep)
{

}

__WEAK void BSP_PowerUpCustom(bool is_deep_sleep)
{

}



void BSP_Power_Up(bool is_deep_sleep)
{
    BSP_PowerUpCustom(is_deep_sleep);
#ifdef SOC_BF0_HCPU

    if (is_deep_sleep)
    {
        //bsp_psram_exit_low_power("psram1");
    }
#elif defined(SOC_BF0_LCPU)
    {
        ;
    }
#endif

}


void BSP_IO_Power_Down(int coreid, bool is_deep_sleep)
{
    BSP_PowerDownCustom(coreid, is_deep_sleep);
#ifdef SOC_BF0_HCPU
    // bsp_psram_enter_low_power("psram1");
#endif  /* SOC_BF0_HCPU */
}


void BSP_CAMERA_PowerUp(void)
{
#ifdef BSP_USING_DCMI
    /*
     * The DVP camera shares PA32/PA33/PA43/PA44/PA45 with the DPI/QADSPI
     * panel. The LCD may have taken those pads over since BSP_PIN_Init()
     * ran, so release the panel and install the DCMI function again before
     * the sensor starts streaming. The two interfaces are mutually
     * exclusive, the panel has to be powered on again afterwards.
     */
#ifdef BSP_USING_LCD
    BSP_LCD_PowerDown();
#endif /* BSP_USING_LCD */
    BSP_CAMERA_Pinmux();
#endif /* BSP_USING_DCMI */

    aw9523_ensure_init();
    /* Camera power on: DVP_PDN=0 (PDN active-high, 0 = normal operation). */
    aw9523_set_pin(AW9523_PIN_DVP_PDN, 0);
    HAL_Delay(20);
}

void BSP_CAMERA_PowerDown(void)
{
    aw9523_ensure_init();
    /* Camera power down: DVP_PDN=1 (AW9523 P0_6). */
    aw9523_set_pin(AW9523_PIN_DVP_PDN, 1);
}