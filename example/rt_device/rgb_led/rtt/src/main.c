#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "drivers/rt_drv_pwm.h"

#define RGB_COLOR   (0x00ff00)

#define RGBLED_NAME    "rgbled"
#define PWM_CHANNEL 1

struct rt_device_pwm *rgbled_device;
struct rt_color
{
    char *color_name;
    uint32_t color;
};

struct rt_color rgb_color_arry[] =
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


void rgb_led_init()
{
    /*rgbled poweron*/
    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO3_3V3, true, true);
    rgbled_device = (struct rt_device_pwm *)rt_device_find(RGBLED_NAME);
    if (!rgbled_device)
    {
        RT_ASSERT(0);
    }
}

void rgb_led_set_color(uint32_t color)
{
    HAL_PIN_Set(PAD_PA32, GPTIM2_CH1, PIN_NOPULL, 1);   // RGB LED
    struct rt_rgbled_configuration configuration;
    configuration.color_rgb = color;
    configuration.channel = PWM_CHANNEL;
    rt_device_control(&rgbled_device->parent, PWM_CMD_SET_COLOR, &configuration);
}

void rgb_led_turnon(uint8_t on)
{
    HAL_PIN_Set(PAD_PA32, GPIO_A32, PIN_NOPULL, 1);   // RGB LED
    rt_pin_mode(RGBLED_CONTROL_PIN, PIN_MODE_OUTPUT);
    if (on)
        rt_pin_write(RGBLED_CONTROL_PIN, PIN_LOW);
    else
        rt_pin_write(RGBLED_CONTROL_PIN, PIN_LOW);
}

void rgb_color_array_display()
{
    uint16_t i;
    rt_kprintf("start display color!\n");
    while (1)
    {
        if (i < sizeof(rgb_color_arry) / sizeof(struct rt_color))
        {
            rt_kprintf("-> %s\n", rgb_color_arry[i].color_name);
            rgb_led_set_color(rgb_color_arry[i].color);
            rt_thread_mdelay(1000);
            rgb_led_turnon(0);
        }
        i++;
        if (i >= 8)
            i = 0;
    }
}


/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    /* Output a message on console using printf function */
    rt_kprintf("Hello world!\n");

    rgb_led_init();
    //rgb_color_auto();
    rgb_color_array_display();
    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(1);
    }
    return 0;
}

