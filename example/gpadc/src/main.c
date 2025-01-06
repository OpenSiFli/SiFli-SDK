#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "dfs_file.h"
#include "drv_flash.h"


#define BSP_BATTERY_DEV_NAME            "bat1"

#ifndef BSP_USING_VBAT
    #error This example need board config enable BSP_USING_VBAT
#endif

static uint32_t get_adc_voltage(void)
{
#ifdef USE_AVERAGE_METHOD
    /* 1, find adc device and enable test channel */
    rt_device_t dev = rt_device_find(BSP_BATTERY_DEV_NAME);
    if (dev != NULL)
    {
        rt_adc_enable((rt_adc_device_t) dev, BSP_VBAT_ADC_CHN);
        rt_kprintf("ADC device %s find, enable channel %d\n", BSP_BATTERY_DEV_NAME, BSP_VBAT_ADC_CHN);
    }
    else
    {
        rt_kprintf("ADC device %s find fail\n", BSP_BATTERY_DEV_NAME);
        return 0;
    }

    /* 2, Read ADC pin and covert to voltage based on 0.1 mv */
    uint32_t voltage = rt_adc_read((rt_adc_device_t) dev, BSP_VBAT_ADC_CHN);
    rt_kprintf("rt_adc_read:  %d \n", voltage);

    /* 3, change voltage base */
    voltage /= 10; // 0.1 mv based to mv

    /* 4, disable test channel */
    rt_adc_disable((rt_adc_device_t) dev, BSP_VBAT_ADC_CHN);
#else
    int ret;
    uint32_t voltage;
    rt_adc_cmd_read_arg_t read_arg;

    read_arg.channel = BSP_VBAT_ADC_CHN;

    /* 1, find adc device and enable test channel */
    rt_device_t dev = rt_device_find(BSP_BATTERY_DEV_NAME);
    if (dev != NULL)
    {
        rt_adc_enable((rt_adc_device_t) dev, read_arg.channel);
        rt_kprintf("ADC device %s find, enable channel %d\n", BSP_BATTERY_DEV_NAME, BSP_VBAT_ADC_CHN);
    }
    else
    {
        rt_kprintf("ADC device %s find fail\n", BSP_BATTERY_DEV_NAME);
        return 0;
    }

    /* 2, get ADC pin and covert to voltage based on 0.1 mv */
    ret = rt_device_control(dev, RT_ADC_CMD_READ, (void *)&read_arg);
    rt_kprintf("rt_device_control:  %d , %d\n", ret, read_arg.value);

    /* 3,  change mv base */
    voltage = read_arg.value / 10;  // 0.1 mv based to mv

    /* 4, disable test channel */
    rt_adc_disable((rt_adc_device_t)dev, read_arg.channel);

#endif
    return voltage;
}

/*For some cases used resistor divider, need conv detect voltage to input voltage */
static uint32_t voltage_div_proc(uint32_t vol)
{
    float inp = (float)vol;
    if (GND2ADC_KOHM > 0)
        return (uint32_t)(inp * (VBAT2ADC_KOHM + GND2ADC_KOHM) / GND2ADC_KOHM);

    return vol;
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    int i;
    uint32_t vbat, vtemp;
    /* Output a message on console using printf function */
    rt_kprintf("Begin user proc\n");

    /* Set GPADC PIN, it should be set at system init, here just as example */

    int pinnum = BSP_VBAT_ADC_PIN;
    if (pinnum >= 96) // Larger than max PA pin number, it's PB pin
    {
        HAL_PIN_Set_Analog(PAD_PB00 + (pinnum - 96), 0);
    }
    else
    {
        HAL_PIN_Set_Analog(PAD_PA00 + pinnum, 1);
    }

    /* Wait a few time to make pin and volatage stable, it depond on hardware setting, remove it if pinmux set at system initial */
    rt_thread_mdelay(500);

    /* Get detect chanel voltage */
    vtemp = get_adc_voltage();
    if (vtemp > 0)
        rt_kprintf("Detect voltage %d mv\n", vtemp);
    else
        rt_kprintf("Get voltage fail\n");

    /* Convert to input voltage if there is divider resistor*/
    vbat = voltage_div_proc(vtemp);
    rt_kprintf("VBAT input %d mv\n", vbat);

    /* Infinite loop */
    rt_kprintf("Begin idle loop\n");
    while (1)
    {
        rt_thread_mdelay(10000);    // Let system breath.
    }
    return 0;
}

