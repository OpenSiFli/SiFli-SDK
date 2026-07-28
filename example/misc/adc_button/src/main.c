/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include "button.h"
#include "sf_type.h"

/* Number of ADC buttons sharing one ADC channel, configured by Kconfig */
#define ADC_BUTTON_NUM      ADC_BUTTON_GROUP1_MAX_NUM
/* ADC button group index, only one group is used in this example */
#define ADC_BUTTON_GROUP    0

/*
 * ADC button event handler.
 *
 * btn_idx is the button index within the group (0, 1, 2 ...), decoded by the
 * button library from the ADC sample value. It is NOT a GPIO number.
 */
static void adc_button_handler(uint8_t group_idx, int32_t btn_idx, button_action_t action)
{
    switch (action)
    {
    case BUTTON_PRESSED:
        rt_kprintf("key %d pressed\n", btn_idx + 1);
        break;

    case BUTTON_RELEASED:
        rt_kprintf("key %d released\n", btn_idx + 1);
        break;

    case BUTTON_CLICKED:
        rt_kprintf("key %d clicked\n", btn_idx + 1);
        break;

    case BUTTON_LONG_PRESSED:
        rt_kprintf("key %d long pressed\n", btn_idx + 1);
        break;

    default:
        break;
    }
}

/*
 * Placeholder handler required by button_init(): it rejects a NULL handler.
 * For ADC buttons this value is replaced inside button_bind_adc_button(),
 * so this function is never actually called.
 */
static void dummy_button_handler(int32_t pin, button_action_t action)
{
}

/* Initialize the ADC buttons. Returns the button id, or a negative value on error. */
static int32_t adc_button_init(void)
{
    int32_t id;
    button_cfg_t cfg;
    adc_button_handler_t handlers[ADC_BUTTON_NUM];

    /* All ADC buttons share one GPIO for the wake-up interrupt */
    cfg.pin = BSP_KEY1_PIN;
    cfg.active_state = BUTTON_ACTIVE_HIGH;
    cfg.mode = PIN_MODE_INPUT;
    cfg.button_handler = dummy_button_handler;

    id = button_init(&cfg);
    if (id < 0)
    {
        rt_kprintf("button_init failed: %d\n", id);
        return -1;
    }

    for (uint8_t i = 0; i < ADC_BUTTON_NUM; i++)
    {
        handlers[i] = adc_button_handler;
    }

    if (button_bind_adc_button(id, ADC_BUTTON_GROUP, ADC_BUTTON_NUM, handlers) != 0)
    {
        rt_kprintf("button_bind_adc_button failed\n");
        return -1;
    }

    if (button_enable(id) != SF_EOK)
    {
        rt_kprintf("button_enable failed\n");
        return -1;
    }

    return id;
}

int main(void)
{
    if (adc_button_init() < 0)
    {
        return -1;
    }

    rt_kprintf("ADC button ready, %d keys. Press a key to see the log.\n", ADC_BUTTON_NUM);

    return 0;
}
