/**
  ******************************************************************************
  * @file   app_button.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "app_common.h"

#define LOG_TAG "app_button"
#include "log.h"

#include "button.h"

#include "drv_gpio.h"

#ifdef BSP_USING_PM
    #include "bf0_pm.h"
#endif

#define SWITCH_ON_PIN (BSP_KEY1_PIN)
#define BUTTON_DOUBLE_CLICKED_INTERVER    (500)

#define BUTTON_FUNC1_LONG_PRESSED_TIMEOUT (500)
#define BUTTON_FUNC2_LONG_PRESSED_TIMEOUT (2500)

typedef enum
{
    KEY_FUNC_1 = 0,
    KEY_FUNC_2,
    KEY_FUNC_MAX,
} key_func_t;


typedef struct
{
    key_func_t last_key;
    uint32_t clicked_tick[KEY_FUNC_MAX];
} key_status_t;

static rt_timer_t                   long_press_timer = NULL;
static key_status_t button_status = {0};

static void button_long_press_timeout(void *param)
{
    extern void pm_shutdown(void);
    LOG_I("button_long_press_timeout");
    pm_shutdown();
}

static void button_long_press_decode(button_action_t action, uint32_t key_func)
{
    uint32_t timeout;
    switch (key_func)
    {
    case KEY_FUNC_1:
    {
        timeout = BUTTON_FUNC1_LONG_PRESSED_TIMEOUT;
        break;
    }
    case KEY_FUNC_2:
    {
        timeout = BUTTON_FUNC2_LONG_PRESSED_TIMEOUT;
        break;
    }
    default:
        timeout = BUTTON_FUNC1_LONG_PRESSED_TIMEOUT;
        ;
    }

    if (BUTTON_LONG_PRESSED == action)
    {
        /* restart long_press_timer */
        if (long_press_timer)
        {
            rt_timer_delete(long_press_timer);
        }
        long_press_timer = rt_timer_create("long_press", button_long_press_timeout, 0,
                                           rt_tick_from_millisecond(timeout), // long pressed detection time is BUTTON_RESTART_DECODE_TIMEOUT+BUTTON_ADV_ACTION_CHECK_DELAY
                                           RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
        RT_ASSERT(long_press_timer);
        rt_timer_start(long_press_timer);
        LOG_I("restart long_press_timer");
    }
    else if (BUTTON_RELEASED == action)
    {
        /* stop long_press_timer */
        if (long_press_timer)
        {
            rt_timer_delete(long_press_timer);
            long_press_timer = NULL;
            LOG_I("stop long_press_timer");
        }
    }
}

#if defined (USING_BUTTON_LIB)
static void button_double_clicked_handle(uint32_t key_func)
{
    uint32_t cur_tick = rt_tick_get();
    uint32_t diff_tick = cur_tick - button_status.clicked_tick[key_func];
    if (diff_tick < BUTTON_DOUBLE_CLICKED_INTERVER)
    {
        LOG_I("%d dobule clicked", key_func);
        switch (key_func)
        {
        case KEY_FUNC_1:
        {
            break;
        }
        case KEY_FUNC_2:
        {
            break;
        }
        default:
            ;
        }
    }
    button_status.clicked_tick[key_func] = cur_tick;
}

static void button_key_func1_handler(button_action_t action, uint32_t key_func)
{
    switch (action)
    {
    case BUTTON_CLICKED:
    {
        button_double_clicked_handle(key_func);
        break;
    }
    case BUTTON_LONG_PRESSED:
    {
        break;
    }
    default:
        ;
    }
}

static void button_key_func2_handler(button_action_t action, uint32_t key_func)
{
    switch (action)
    {
    case BUTTON_CLICKED:
    {
        button_double_clicked_handle(key_func);
        break;
    }
    case BUTTON_LONG_PRESSED:
    {
        break;
    }
    default:
        ;
    }
}

static void button_event_handler(button_action_t action, uint32_t key_func)
{
    LOG_I("button: %d %d", action, key_func);
    switch (key_func)
    {
    case KEY_FUNC_1:
    {
        button_key_func1_handler(action, key_func);
        break;
    }
    case KEY_FUNC_2:
    {
        button_key_func2_handler(action, key_func);
        break;
    }
    default:
        ;
    }
}
#endif

static void button_handler(int32_t pin, button_action_t action)
{
    return;
}


#ifdef USING_ADC_BUTTON
static void adc_button_handler(uint8_t group_idx, int32_t pin, button_action_t button_action)
{
    rt_kprintf("group:%d,%d,%d\n", group_idx, pin, button_action);
    button_long_press_decode(button_action, pin);
#if defined (USING_BUTTON_LIB)
    button_event_handler(button_action, pin);
#endif
}

#endif

static void button_config(uint32_t key_pin, button_active_state_t active_state)
{
#ifdef BSP_USING_PM
    int8_t wakeup_pin;
    uint16_t gpio_pin;
    GPIO_TypeDef *gpio;
#endif /* BSP_USING_PM */

    sf_err_t err;
    button_cfg_t cfg = { 0 };
    cfg.pin = key_pin;
    cfg.active_state = active_state;
    cfg.mode = PIN_MODE_INPUT;
    cfg.button_handler = button_handler;
    int32_t id = button_init(&cfg);
    RT_ASSERT(id >= 0);
#ifdef USING_ADC_BUTTON
    adc_button_handler_t handler[3] = {adc_button_handler, adc_button_handler, adc_button_handler};
    err = button_bind_adc_button(id, 0, ADC_BUTTON_GROUP1_MAX_NUM, &handler[0]);
    RT_ASSERT(0 == err);
#endif
    err = button_enable(id);
    RT_ASSERT(SF_EOK == err);

#ifdef BSP_USING_PM
    gpio = GET_GPIO_INSTANCE(key_pin);
    gpio_pin = GET_GPIOx_PIN(key_pin);

    wakeup_pin = HAL_HPAON_QueryWakeupPin(gpio, gpio_pin);
    if (wakeup_pin >= 0)
    {
        pm_enable_pin_wakeup(wakeup_pin, AON_PIN_MODE_DOUBLE_EDGE);
    }
#endif /* BSP_USING_PM */
    return;
}

#if defined (USING_BUTTON_LIB)
static int init_pin(void)
{
    button_active_state_t active_state;
#if (SWITCH_ON_PIN < GPIO1_PIN_NUM)
    /* SWITCH_ON PIN is by HPSYS side */
#if defined (BSP_KEY1_ACTIVE_HIGH) && (BSP_KEY1_ACTIVE_HIGH == 1)
    active_state = BUTTON_ACTIVE_HIGH;
#else
    active_state = BUTTON_ACTIVE_LOW;
#endif
    button_config(SWITCH_ON_PIN, active_state);
#endif /* SWITCH_ON_PIN < GPIO1_PIN_NUM */
    return 0;
}

INIT_PRE_APP_EXPORT(init_pin);
#endif


