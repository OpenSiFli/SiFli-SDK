/*
 * SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include "button.h"
#include <board.h>
#include "bf0_hal.h"
#include "composite_device.h"

static void button_event_handler(int32_t pin, button_action_t action)
{
    rt_kprintf("button:%d,%d\n", pin, action);
    
    // Only respond to button click events
    if (action == BUTTON_CLICKED)
    {
        // Check if USB composite device is ready before sending data
        if (!composite_device_is_ready(0))
        {
            rt_kprintf("USB device not ready yet, please wait...\n");
            return;
        }

        if (pin == BSP_KEY1_PIN) // KEY1
        {
            rt_kprintf("KEY1 clicked, sending 'SiFli'\n");
            hid_keyboard_send_string(0, "SiFli");
        }
        else if (pin == BSP_KEY2_PIN) // KEY2
        {
            rt_kprintf("KEY2 clicked, sending 'CherryUSB'\n");
            hid_keyboard_send_string(0, "CherryUSB");
        }
    }
}

int main(void)
{
    rt_kprintf("USB Composite Device Demo (MSC + CDC ACM + HID Keyboard)!\n");

    // Configure button 1
    button_cfg_t cfg;
    cfg.pin = BSP_KEY1_PIN;
    cfg.active_state = BUTTON_ACTIVE_HIGH;
    cfg.mode = PIN_MODE_INPUT;
    cfg.button_handler = button_event_handler;
    int32_t id1 = button_init(&cfg);
    RT_ASSERT(id1 >= 0);
    RT_ASSERT(SF_EOK == button_enable(id1));

    cfg.pin = BSP_KEY2_PIN;
    cfg.active_state = BUTTON_ACTIVE_HIGH;
    cfg.mode = PIN_MODE_INPUT;
    cfg.button_handler = button_event_handler;
    int32_t id2 = button_init(&cfg);
    RT_ASSERT(id2 >= 0);
    RT_ASSERT(SF_EOK == button_enable(id2));

    // Initialize USB composite device
    composite_device_init(0, (uintptr_t)USBC_BASE);

    rt_kprintf("Composite device initialized. Press buttons to send text:\n");
    rt_kprintf("KEY1: Send 'SiFli'\n");
    rt_kprintf("KEY2: Send 'CherryUSB'\n");

    while (1)
    {
        rt_thread_mdelay(1000);
    }

    return RT_EOK;
}