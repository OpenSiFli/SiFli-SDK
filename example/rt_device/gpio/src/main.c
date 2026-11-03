/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "board.h"

#define DBG_TAG "gpio"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef SF32LB52X
    #define Pin_Out 41
    #define Pin_In 42
    #define hwp_gpio hwp_gpio1
#elif defined(SF32LB58X)
    #define Pin_Out 124
    #define Pin_In 125
    #define hwp_gpio hwp_gpio2
#elif defined(SF32LB56X)
    #define Pin_Out 20
    #define Pin_In 12
    #define hwp_gpio hwp_gpio1
#elif defined(SF32LB57X)
    #define Pin_Out 41
    #define Pin_In 42
    #define hwp_gpio hwp_gpio1
#endif

rt_device_t device;

void irq_handler(void *args)
{
    //set your own irq handle
    rt_kprintf("Interrupt occurred!\n");

    //read Pin_Out
    struct rt_device_pin_status st;
    st.pin = Pin_Out;
    rt_device_read(device, 0, &st, sizeof(struct rt_device_pin_status));
    rt_kprintf("Pin_Out %d has been toggle, value = %d\n", Pin_Out, st.status);

    //read Pin_In
    st.pin = Pin_In;
    rt_device_read(device, 0, &st, sizeof(struct rt_device_pin_status));
    rt_kprintf("Pin_In %d, value = %d\n", Pin_In, st.status);
    rt_kprintf(" \n");
}

void gpio_init(void)
{
    //1. pin mux
#ifdef SF32LB52X
    HAL_PIN_Set(PAD_PA00 + Pin_Out, GPIO_A0 + Pin_Out, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA00 + Pin_In, GPIO_A0 + Pin_In, PIN_PULLDOWN, 1);
#elif defined(SF32LB58X)
    HAL_PIN_Set(PAD_PB00 + Pin_Out - 96, GPIO_B0 + Pin_Out - 96, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB00 + Pin_In - 96, GPIO_B0 + Pin_In - 96, PIN_PULLDOWN, 0);
#elif defined(SF32LB56X)
    HAL_PIN_Set(PAD_PA00 + Pin_Out, GPIO_A0 + Pin_Out, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA00 + Pin_In, GPIO_A0 + Pin_In, PIN_PULLDOWN, 1);
#elif defined(SF32LB57X)
    HAL_PIN_Set(PAD_PA00 + Pin_Out, GPIO_A0 + Pin_Out, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA00 + Pin_In, GPIO_A0 + Pin_In, PIN_PULLDOWN, 1);
#endif



    // 2. gpio init

    if (!device)
    {
        LOG_I("Find device pin fail\n");
    }
    rt_device_open(device, RT_DEVICE_OFLAG_RDWR);

    //set pin to PIN_MODE_OUTPUT
    struct rt_device_pin_mode m;
    m.pin = Pin_Out;
    m.mode = PIN_MODE_OUTPUT;
    rt_device_control(device, 0, &m);

    //set pin to PIN_MODE_INPUT
    m.pin = Pin_In;
    m.mode = PIN_MODE_INPUT;
    rt_device_control(device, 0, &m);

    //set irq mode
    rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING_FALLING, irq_handler, (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 1);
}

void gpio_test(void)
{
    struct rt_device_pin_status st;

    //set Pin_Out output 0
    st.pin = Pin_Out;
    st.status = 0;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));
    rt_thread_mdelay(1000);

    //set Pin_Out output 1
    st.pin = Pin_Out;
    st.status = 1;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));
    rt_thread_mdelay(1000);
}

int main(void)
{
    rt_kprintf("Start gpio rtt demo!\n"); // Output a start message on console using rt_kprintf function
    device = rt_device_find("pin");
    gpio_init();
    while (1)
    {
        gpio_test();
    }
    return 0;
}
