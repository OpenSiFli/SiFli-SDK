/*
 * SPDX-FileCopyrightText: 2025-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "drivers/rt_drv_pwm.h"

#define RGBLED_NAME    "rgbled"


struct rt_device *rgbled_device;
struct rt_color
{
    char *color_name;
    uint32_t color;
};

static struct rt_color rgb_color_arry[] =
{
    {"black", 0x000000},
    {"blue", 0x0000ff},
    {"green", 0x00ff00},
    {"cyan",  0x00ffff},
    {"red", 0xff0000},
    {"purple", 0xff00ff},
    {"yellow", 0xffff00},
    {"white", 0xffffff}
};


static void rgb_led_init()
{
#ifdef SF32LB52X
    HAL_PIN_Set(PAD_PA32, GPTIM2_CH1, PIN_NOPULL, 1);   // pwmt2_cc1
#elif defined SF32LB58X
    HAL_PIN_Set(PAD_PB39, GPTIM3_CH4, PIN_NOPULL, 0);   // pwmt3_cc4
#elif defined SF32LB56X
    HAL_PIN_Set(PAD_PB09, GPTIM3_CH4, PIN_NOPULL, 0);   // pwmt3_cc4
#endif
    /*rgbled poweron*/
#ifdef SF32LB52X
    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO3_3V3, true, true);
#endif
    rgbled_device = rt_device_find(RGBLED_NAME);//find rgb
    if (!rgbled_device)
    {
        RT_ASSERT(0);
    }
}

static void rgb_led_set_color(uint32_t color)
{
    struct rt_rgbled_configuration configuration;
    configuration.color_rgb = color;
    rt_device_control(rgbled_device, PWM_CMD_SET_COLOR, &configuration);
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rgb_led_init();

    rt_kprintf("start display color!\n");
    /* Infinite loop */
    while (1)
    {
        for (int i = 0; i < sizeof(rgb_color_arry) / sizeof(rgb_color_arry[0]); i++)
        {
            rt_kprintf("-> %s\n", rgb_color_arry[i].color_name);
            rgb_led_set_color(rgb_color_arry[i].color);
            rt_thread_mdelay(1000);
        }
    }
    return 0;
}
