/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "board.h"
#define DBG_TAG "adc"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
/* adc example for RT-Thread based platform -----------------------------------------------*/
#include "bf0_sys_cfg.h"
#define ADC_DEV_NAME        "bat1"      /* ADC name */
#if defined(SF32LB52X)
    #define ADC_DEV_CHANNEL     7           /* ADC channel 7 */
#elif defined(SF32LB56X)
    #define ADC_DEV_CHANNEL     1           /* ADC channel 1 */
#elif defined(SF32LB58X)
    #define ADC_DEV_CHANNEL     2           /* ADC channel 2*/
#else
    #define ADC_DEV_CHANNEL     7           /* ADC channel 7*/
#endif
//#define REFER_VOLTAGE       330         /* referencen voltage 3.3V*/
static rt_device_t s_adc_dev;
static rt_adc_cmd_read_arg_t read_arg;
static rt_err_t adc_example_init(void)
{
    rt_err_t r;
    /* set pinmux of channel 0 to analog input */
#if defined(SF32LB56X)
    HAL_PIN_Set_Analog(PAD_PB23, 0);
#elif defined(SF32LB58X)
    HAL_PIN_Set_Analog(PAD_PB34, 0);
#endif
    /* find device */
    s_adc_dev = rt_device_find(ADC_DEV_NAME);
    if (s_adc_dev == RT_NULL)
    {
        rt_kprintf("find %s failed\n", ADC_DEV_NAME);
        return -RT_ERROR;
    }
    /* set channel 0*/
    read_arg.channel = ADC_DEV_CHANNEL;
    r = rt_adc_enable((rt_adc_device_t)s_adc_dev, read_arg.channel);
    if (r != RT_EOK)
    {
        rt_kprintf("rt_adc_enable failed:%d\n", r);
        return r;
    }
    return RT_EOK;
}

void adc_example(void)
{
    /* will call funtion: sifli_adc_control, and only read once from adc */
    // r = rt_device_control(s_adc_dev, RT_ADC_CMD_READ, &read_arg.channel);
    // LOG_I("adc channel:%d,value:%d", read_arg.channel, read_arg.value); /* (0.1mV), 20846 is 2084.6mV or 2.0846V */
    /* another way to read adc, will call funntion:sifli_get_adc_value,read 22 times from adc and get the average */
    rt_uint32_t value = rt_adc_read((rt_adc_device_t)s_adc_dev, ADC_DEV_CHANNEL);
    LOG_I("rt_adc_read:%d,value:%d", read_arg.channel, value); /* (0.1mV), 20700 is 2070mV or 2.070V */
}
/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_kprintf("Start adc demo!\n");
    if (adc_example_init() != RT_EOK)
    {
        return -RT_ERROR;
    }
    while (1)
    {
        adc_example();
        rt_thread_mdelay(1000);
    }
    return RT_EOK;
}

