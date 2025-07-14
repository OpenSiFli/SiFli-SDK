#include <rtthread.h>
#include <string.h>
#include <stdio.h>
#include <board.h>
#include <bf0_hal_tim.h>
#include <drivers/rt_drv_encoder.h>
#include "drv_io.h"

#define ENCODER_DEVICE_NAME1 "encoder1"
#define ENCODER_DEVICE_NAME2 "encoder2"

struct rt_device *encoder_device1;
struct rt_device *encoder_device2;


rt_err_t encoder_example_init(void)
{
    rt_err_t result1;
    rt_err_t result2;



#ifdef SF32LB52X
    HAL_PIN_Set(PAD_PA37, GPTIM2_CH1, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA38, GPTIM2_CH2, PIN_PULLUP, 1);

    HAL_PIN_Set(PAD_PA27, GPTIM1_CH1, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA20, GPTIM1_CH2, PIN_PULLUP, 1);


#elif defined SF32LB58X
    HAL_PIN_Set(PAD_PA82, GPTIM1_CH1, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA51, GPTIM1_CH2, PIN_PULLUP, 1);
#endif

    encoder_device1 = rt_device_find(ENCODER_DEVICE_NAME1);


    if (encoder_device1 == RT_NULL)
    {
        rt_kprintf("Failed to find %s device\n", ENCODER_DEVICE_NAME1);
        return -RT_ERROR;
    }

    encoder_device2 = rt_device_find(ENCODER_DEVICE_NAME2);
    if (encoder_device2 == RT_NULL)
    {
        rt_kprintf("Failed to find %s device\n", ENCODER_DEVICE_NAME2);
        return -RT_ERROR;
    }

    rt_kprintf("Found encoder devices: %s and %s\n", ENCODER_DEVICE_NAME1, ENCODER_DEVICE_NAME2);
    // Starting encoder
    struct rt_encoder_configuration config1;
    struct rt_encoder_configuration config2;


    config1.channel = GPT_CHANNEL_ALL;
    config2.channel = GPT_CHANNEL_ALL;

    result1 = rt_device_control((struct rt_device *)encoder_device1, PULSE_ENCODER_CMD_ENABLE, (void *)&config1);//使能

    if (result1 != RT_EOK)
    {
        rt_kprintf("Failed to enable encoder\n");
        return -RT_ERROR;
    }


    result2 = rt_device_control((struct rt_device *)encoder_device2, PULSE_ENCODER_CMD_ENABLE, (void *)&config2);//使能

    if (result2 != RT_EOK)
    {
        rt_kprintf("Failed to enable encoder\n");
        return -RT_ERROR;
    }



    rt_kprintf("Encoder devices initialized successfully\n");
    return RT_EOK;
}

void Get_count(void)
{

    rt_err_t result1;
    rt_err_t result2;

    struct rt_encoder_configuration config_count1;
    struct rt_encoder_configuration config_count2;

    config_count1.get_count = 0;
    config_count2.get_count = 0;




    result2 = rt_device_control((struct rt_device *)encoder_device2, PULSE_ENCODER_CMD_GET_COUNT, (void *)&config_count2);

    if (result2 != RT_EOK)
    {
        rt_kprintf("Failed to get encoder count\n");
    }
    else
    {
        rt_kprintf("encoder2_count:%d\n", config_count2.get_count);
    }


    // Gets the current count value
    result1 = rt_device_control((struct rt_device *)encoder_device1, PULSE_ENCODER_CMD_GET_COUNT, (void *)&config_count1);

    if (result1 != RT_EOK)
    {
        rt_kprintf("Failed to get encoder count\n");
    }
    else
    {
        rt_kprintf("encoder1_count:%d\n", config_count1.get_count);
    }
}


int main(void)
{

    rt_err_t result;
    result = encoder_example_init();
    if (result != RT_EOK)
    {
        rt_kprintf("Failed encoder_example_init\n");
    }
    else
    {
        rt_kprintf("succeed encoder_example_init\n");
    }
    rt_kprintf("Start Get_count!\n");
    while (1)
    {
        Get_count();
        rt_thread_mdelay(1000);
    }
    return 0;

}