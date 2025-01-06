#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"
#include "board.h"
#include "bf0_sys_cfg.h"
#include <rtdbg.h>

#define DBG_TAG "adc"
#define DBG_LVL DBG_LOG
#define ADC_DEV_NAME "bat1"      /* ADC name */
#define ADC_DEV_CHANNEL 7           /* ADC channe7 */


/* GPIO_TEST -----------------------------------------------*/
uint32_t pin_out[] =
{
    42, 40, 3, 44,
    38, 36, 33, 10,
    30, 1, 31, 5,
    7, 9
};


uint32_t pin_in[] =
{
    41, 39, 11, 43,
    37, 35, 34, 25,
    29, 0, 2, 4,
    6, 8
};

int pin_out_num = sizeof(pin_out) / sizeof(pin_out[0]);
int pin_in_num = sizeof(pin_in) / sizeof(pin_in[0]);
int pin_num = 0;



void pinmux(void)
{
    //out
    for (int i = 0; i < pin_num; i++)
    {
        HAL_PIN_Set(PAD_PA00 + pin_out[i], GPIO_A0 + pin_out[i], PIN_NOPULL, 1);
    }

    //in
    for (int i = 0; i < pin_num; i++)
    {
        HAL_PIN_Set(PAD_PA00 + pin_in[i], GPIO_A0 + pin_in[i], PIN_NOPULL, 1);
    }
}


void gpio_init(void)
{
    HAL_StatusTypeDef ret;

    //1. pin mux
    pinmux();


    // 2. gpio init
    HAL_RCC_EnableModule(RCC_MOD_GPIO1); // GPIO clock enable
    GPIO_InitTypeDef GPIO_InitStruct;


    for (int i = 0; i < pin_num; i++)
    {
        GPIO_InitStruct.Pin = pin_out[i];
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);
        HAL_GPIO_WritePin(hwp_gpio1, pin_out[i], GPIO_PIN_RESET);
    }


    // pin_in
    GPIO_InitStruct.Pin = 26 ;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);

    for (int i = 0; i < pin_num; i++)
    {
        GPIO_InitStruct.Pin = pin_in[i];
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);
    }
}


void gpio_test(void)
{
    if (pin_out_num == pin_in_num)
    {
        pin_num = pin_out_num;
    }
    else
    {
        rt_kprintf("pin_out[] or pin_in[] wrong! please check\n");
    }

    gpio_init();

    bool ret = true;
    int val = 0;

    if (1 == HAL_GPIO_ReadPin(hwp_gpio1, 26))
    {
        rt_kprintf("Init FAILED! Pin_GND or Pin_IN 26 error\n");
        ret = false;
    }

    for (int i = 0; i < pin_num; i++)
    {

        val = HAL_GPIO_ReadPin(hwp_gpio1, pin_in[i]); //0
        if (1 == val)
        {
            rt_kprintf("Init FAILED! Pin_OUT %d or Pin_IN %d error\n", pin_out[i], pin_in[i]);
            val = 0;
            ret = false;
        }
    }

    for (int i = 0; i < pin_num; i++)
    {

        val = HAL_GPIO_ReadPin(hwp_gpio1, pin_in[i]); //0
        HAL_GPIO_TogglePin(hwp_gpio1, pin_out[i]);
        val = (val != HAL_GPIO_ReadPin(hwp_gpio1, pin_in[i])); //1
        if (val == false)
        {
            rt_kprintf("Test ERROR! Pin_OUT %d or Pin_IN %d error\n", pin_out[i], pin_in[i]);
            ret = false;
        }

        if (1 == HAL_GPIO_ReadPin(hwp_gpio1, 26))
        {
            rt_kprintf("Received ERROR! Pin_IN 26 error\n");
            ret = false;
        }

        for (int k = 0; k < i; k++)
        {
            val = HAL_GPIO_ReadPin(hwp_gpio1, pin_in[k]);
            if (1 == val)
            {
                rt_kprintf("Received ERROR! Pin_IN %d error\n", pin_in[k]);
                val = 0;
                ret = false;
            }
        }

        for (int k = (i + 1);  k < pin_num; k++)
        {
            val = HAL_GPIO_ReadPin(hwp_gpio1, pin_in[k]);
            if (1 == val)
            {
                rt_kprintf("Received ERROR! Pin_IN %d error\n", pin_in[k]);
                val = 0;
                ret = false;
            }
        }
        HAL_GPIO_TogglePin(hwp_gpio1, pin_out[i]);
    }
    if (ret == true)rt_kprintf("gpio_test PASS\n");
    else rt_kprintf("gpio_test FAIL\n");
}
MSH_CMD_EXPORT(gpio_test, test GPIO);


/* VBAT_TEST -----------------------------------------------*/
static rt_device_t s_adc_dev;
static rt_adc_cmd_read_arg_t read_arg;
static rt_uint32_t value;
static float value_last;

void adc_example(void)
{
    rt_err_t r;
    s_adc_dev = rt_device_find(ADC_DEV_NAME);
    read_arg.channel = ADC_DEV_CHANNEL;
    r = rt_adc_enable((rt_adc_device_t)s_adc_dev, read_arg.channel);
    value = rt_adc_read((rt_adc_device_t)s_adc_dev, ADC_DEV_CHANNEL);
    value_last = (float)value / 10000;
    rt_adc_disable((rt_adc_device_t)s_adc_dev, read_arg.channel);
}

void vbat_test(void)
{
    adc_example();
    rt_kprintf("value:%.4fV\n", value_last);
}
MSH_CMD_EXPORT(vbat_test, say value_last to RT - Thread);


/* OPEN UART2 -----------------------------------------------*/
void uart_init(void)
{
    /* 1, pinmux set to uart mode */
    HAL_PIN_Set(PAD_PA20, USART2_RXD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA27, USART2_TXD, PIN_PULLUP, 1);
}

/* main() -----------------------------------------------*/
int main(void)
{
    rt_kprintf("Start 52x_mode_test\n");
    rt_kprintf("Commond list\n");
    rt_kprintf("<gpio_test>\n");
    rt_kprintf("<vbat_test>\n");

    uart_init();

    while (1)
    {
        rt_thread_mdelay(5000);
    }

    return 0;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/




