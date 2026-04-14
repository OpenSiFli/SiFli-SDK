/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtconfig.h"
#include "bf0_hal.h"
#include "rtthread.h"
#include <stdint.h>

#include "ll_pinmux.h"
#include "ll_gpio.h"

#ifndef SF32LB52X
    #error "This example currently supports SF32LB52x only."
#endif

#define LL_GPIO_OUT_PIN 41U
#define LL_GPIO_IN_PIN 42U
#define LL_GPIO_IRQn GPIO1_IRQn

static volatile uint32_t g_irq_count;

/**
 * @brief Initialize GPIO output, GPIO interrupt source, and NVIC.
 */
static void ll_gpio_example_board_init(void)
{
    ll_gpio_irq_config_t irq_cfg = {
        .type = LL_GPIO_IRQ_TYPE_EDGE,
        .polarity = LL_GPIO_IRQ_POL_BOTH,
    };
    ll_pinmux_pad_config_t out_pad_cfg = {
        .fsel = 0U,
        .pull = LL_PINMUX_PULL_UP,
        .input_en = LL_PINMUX_INPUT_ENABLE,
        .input_type = LL_PINMUX_INPUT_SCHMITT,
        .slew = LL_PINMUX_SLEW_FAST,
        .drive = LL_PINMUX_DRIVE_1,
    };
    ll_pinmux_pad_config_t in_pad_cfg = {
        .fsel = 0U,
        .pull = LL_PINMUX_PULL_DOWN,
        .input_en = LL_PINMUX_INPUT_ENABLE,
        .input_type = LL_PINMUX_INPUT_SCHMITT,
        .slew = LL_PINMUX_SLEW_FAST,
        .drive = LL_PINMUX_DRIVE_1,
    };

    ll_pinmux_config_pad(hwp_pinmux1, 13U + LL_GPIO_OUT_PIN, &out_pad_cfg);
    ll_pinmux_config_pad(hwp_pinmux1, 13U + LL_GPIO_IN_PIN, &in_pad_cfg);

    HAL_RCC_EnableModule(RCC_MOD_GPIO1);

    ll_gpio_pa_set_low(LL_GPIO_OUT_PIN);
    ll_gpio_pa_enable_output(LL_GPIO_OUT_PIN);

    ll_gpio_pa_config_irq_trigger(LL_GPIO_IN_PIN, &irq_cfg);
    ll_gpio_pa_clear_irq_pending(LL_GPIO_IN_PIN);
    ll_gpio_pa_enable_irq(LL_GPIO_IN_PIN);

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
    HAL_NVIC_SetPriority(LL_GPIO_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(LL_GPIO_IRQn);
}

/**
 * @brief GPIO1 interrupt handler.
 */
void GPIO1_IRQHandler(void)
{
    uint32_t in_level;
    uint32_t out_level;

    rt_interrupt_enter();

    if (ll_gpio_pa_is_irq_pending(LL_GPIO_IN_PIN) != 0U)
    {
        ll_gpio_pa_clear_irq_pending(LL_GPIO_IN_PIN);

        in_level = (ll_gpio_pa_is_input_high(LL_GPIO_IN_PIN) != 0U) ? 1U : 0U;
        out_level = (ll_gpio_pa_is_input_high(LL_GPIO_OUT_PIN) != 0U) ? 1U : 0U;
        g_irq_count++;

        rt_kprintf("LL GPIO IRQ count=%lu, out=%lu, in=%lu\n",
                   (unsigned long)g_irq_count, (unsigned long)out_level,
                   (unsigned long)in_level);
    }

    rt_interrupt_leave();
}

/**
 * @brief Application entry for LL GPIO interrupt example.
 * @return This function does not return.
 */
int main(void)
{
    rt_kprintf("LL GPIO IRQ example started.\n");
    rt_kprintf("Please connect PA41 to PA42 with a jumper.\n");

    ll_gpio_example_board_init();

    while (1)
    {
        ll_gpio_pa_toggle(LL_GPIO_OUT_PIN);
        rt_thread_mdelay(1000);
    }
}
