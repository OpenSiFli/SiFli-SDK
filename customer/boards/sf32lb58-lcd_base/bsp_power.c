/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_board.h"

#define SD1_RESET_PIN       (49)
#define SD1_EN_PIN          (80)
#define SD1_VDD_PIN         (74)

void BSP_GPIO_Set(int pin, int val, int is_porta)
{
    GPIO_TypeDef *gpio = (is_porta) ? hwp_gpio1 : hwp_gpio2;
    GPIO_InitTypeDef GPIO_InitStruct;

    // set sensor pin to output mode
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &GPIO_InitStruct);

    // set sensor pin to high == power on sensor board
    HAL_GPIO_WritePin(gpio, pin, (GPIO_PinState)val);
}


void BSP_Power_Up(bool is_deep_sleep)
{
#ifdef SOC_BF0_HCPU

    if (is_deep_sleep)
    {
        // Replace with API that is OS-independent.
        //rt_psram_exit_low_power("psram0");
    }
#elif defined(SOC_BF0_LCPU)
    {
        ;
    }
#endif

}

void BSP_IO_Power_Down(int coreid, bool is_deep_sleep)
{
    int i;
#ifdef SOC_BF0_HCPU
    if (coreid == CORE_ID_HCPU)
    {
        // Replace with API that is OS-independent.
        // if (is_deep_sleep)
        //rt_psram_enter_low_power("psram0");
    }
#else
    {
        ;
    }
#endif
}

void sd1_pinmux_config()
{
    HAL_PIN_Set(PAD_PA39, SD1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA41, SD1_DIO0, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA30, SD1_DIO1, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA36, SD1_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA40, SD1_DIO3, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA38, SD1_DIO4, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA37, SD1_DIO5, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA35, SD1_DIO6, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA33, SD1_DIO7, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA34, SD1_CMD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA49, GPIO_A49, PIN_PULLUP, 1);     // SD1 RESET, need set 0 first?
    HAL_PIN_Set(PAD_PA80, GPIO_A80, PIN_PULLUP, 1);     // SD1 EN
}


void BSP_SD_PowerUp(void)
{
#ifdef PMIC_CTRL_ENABLE
    BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_5, 1, 1); //LCD_1V8 power
    BSP_PMIC_Control(PMIC_OUT_LDO33_VOUT, 1, 1);    //LCD_3V3 power
#endif /* PMIC_CTRL_ENABLE */

    BSP_GPIO_Set(SD1_EN_PIN, 1, 1);
    BSP_GPIO_Set(SD1_RESET_PIN, 1, 1);
    BSP_GPIO_Set(SD1_VDD_PIN, 1, 1);
    sd1_pinmux_config();
}

void BSP_SD_PowerDown(void)
{
    BSP_GPIO_Set(SD1_EN_PIN, 0, 1);
    BSP_GPIO_Set(SD1_RESET_PIN, 0, 1);
    BSP_GPIO_Set(SD1_VDD_PIN, 0, 1);
}

void sd2_pinmux_config()
{
    HAL_PIN_Set(PAD_PA70, SD2_CMD,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA75, SD2_DIO1, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA76, SD2_DIO0, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA77, SD2_CLK,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA79, SD2_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA81, SD2_DIO3, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA58, GPIO_A58, PIN_NOPULL, 1);     // SD1 card detect pin
}

void BSP_SD2_PowerUp(void)
{
    sd2_pinmux_config();
}

void BSP_SD2_PowerDown(void)
{

}
