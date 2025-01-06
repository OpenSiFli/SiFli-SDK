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
#define ADC_DEV_CHANNEL     0           /* ADC channe0 */
//#define REFER_VOLTAGE       330         /* referencen voltage 3.3V*/
static rt_device_t s_adc_dev;
static rt_adc_cmd_read_arg_t read_arg;
void adc_example(void)
{
    rt_err_t r;
    /* set pinmux of channel 0 to analog input */
    HAL_PIN_Set_Analog(PAD_PA28, 1);
    /* find device */
    s_adc_dev = rt_device_find(ADC_DEV_NAME);
    /* set channel 0*/
    read_arg.channel = ADC_DEV_CHANNEL;
    r = rt_adc_enable((rt_adc_device_t)s_adc_dev, read_arg.channel);
    /* will call funtion: sifli_adc_control, and only read once from adc */
    // r = rt_device_control(s_adc_dev, RT_ADC_CMD_READ, &read_arg.channel);
    // LOG_I("adc channel:%d,value:%d", read_arg.channel, read_arg.value); /* (0.1mV), 20846 is 2084.6mV or 2.0846V */
    /* another way to read adc, will call funntion:sifli_get_adc_value,read 22 times from adc and get the average */
    rt_uint32_t value = rt_adc_read((rt_adc_device_t)s_adc_dev, ADC_DEV_CHANNEL);
    LOG_I("rt_adc_read:%d,value:%d", read_arg.channel, value); /* (0.1mV), 20700 is 2070mV or 2.070V */
    /* disable adc */
    rt_adc_disable((rt_adc_device_t)s_adc_dev, read_arg.channel);
}
/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_kprintf("Start adc demo!\n");
    //HAL_Delay(250);
    adc_example();

    rt_kprintf("spi adc end!\n");
    while (1)
    {
        // adc_example();
        rt_thread_mdelay(5000);
        //rt_kprintf("__main loop__\r\n");
    }
    return RT_EOK;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

