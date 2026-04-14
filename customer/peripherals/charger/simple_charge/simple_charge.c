/*
 * SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// simple_charge.c

#include "rtthread.h"
#include "rtdevice.h"
#include "charge.h"
#include "board.h"
#include "drv_gpio.h"

static rt_charge_device_t simple_charge_device;

void simple_charge_input_handle(void *args)
{
    static uint32_t last_time;

    uint32_t cur_time = rt_tick_get();
    uint32_t tick = (cur_time - last_time + UINT32_MAX + 1) & UINT32_MAX;

    if (tick > 150)
    {
        rt_charge_event_notify(RT_CHARGE_EVENT_DETECT);
    }

    last_time = cur_time;
    return;
}

static void simple_charge_pin_init(void)
{
#if defined(BSP_USING_CHARGER_DETECT)
    rt_pin_mode(BSP_CHARGER_INT_PIN, PIN_MODE_INPUT);

    rt_pin_attach_irq(BSP_CHARGER_INT_PIN, PIN_IRQ_MODE_RISING_FALLING,
                      (void *)simple_charge_input_handle,
                      (void *)(rt_uint32_t)BSP_CHARGER_INT_PIN);

    rt_pin_irq_enable(BSP_CHARGER_INT_PIN, 1);
#endif
}

int simple_get_detect_status()
{
    int status = 0xff;
#ifdef BSP_CHARGER_INT_PIN_ACTIVE_HIGH
    status = rt_pin_read(BSP_CHARGER_INT_PIN);
#else
    status = !rt_pin_read(BSP_CHARGER_INT_PIN);
#endif
    return status;
}

static rt_err_t simple_charge_control(rt_charge_device_t *charge, int cmd, void *args)
{
    rt_charge_err_t ret = RT_CHARGE_EOK;

    switch (cmd)
    {
    case RT_CHARGE_GET_STATUS:
    {
        uint8_t *status = (uint8_t *)args;
        *status = simple_get_detect_status();
    }
    break;

    case RT_CHARGE_GET_DETECT_STATUS:
    {
        uint8_t *status = (uint8_t *)args;
        *status = simple_get_detect_status();
    }
    break;
    case RT_CHARGE_SET_CC_CURRENT:
    {
        /*This charging chip driver does not support setting the charging current.*/
    }
    break;
    default:
        ret = RT_CHARGE_ERROR_UNSUPPORTED;
        break;
    }

    return ret;
}

static const struct rt_charge_ops simple_charge_ops =
{
    .control = simple_charge_control,
};

int simple_charge_init(void)
{
    simple_charge_pin_init();

    rt_charge_register(&simple_charge_device, &simple_charge_ops, RT_NULL);
    return RT_EOK;
}

INIT_PREV_EXPORT(simple_charge_init);