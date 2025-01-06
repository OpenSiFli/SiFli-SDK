/**
  ******************************************************************************
  * @file   button.c
  * @author Sifli software development team
  * @brief Sibles button library source.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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


#include <rtthread.h>
#include <rtdevice.h>
#include <rtdef.h>
#include <string.h>
#include "board.h"
#include "button.h"
#include "log.h"
#include "drv_gpio.h"

#ifdef BUTTON_SERVICE_ENABLED
    #include "stdio.h"
    #include "data_service_provider.h"
#endif /* BUTTON_SERVICE_ENABLED */

/** default detection delay in millisecond */
#define BUTTON_DETECTION_DELAY   (1)
#define GROUP_PIN_NUM    (64)
#define MAX_GROUP_NUM    (4)
#define MAX_PIN          (MAX_GROUP_NUM * GROUP_PIN_NUM - 1)
#define BUTTON_VOLT_VALID_RANGE (50)


#ifndef BUTTON_SERVICE_MAX_CLIENT_NUM
    #define BUTTON_SERVICE_MAX_CLIENT_NUM (1)
#endif /* BUTTON_SERVICE_MAX_CLIENT_NUM */

typedef struct
{
    bool valid;           /**< if cfg is valid  */
    bool enabled;         /**< if button is enabled */
    int last_state;
    uint16_t debounce_cnt;  /**< >0: debouncing is ongoing, ==0: no debouncing is ongoing*/
    struct rt_timer timer;  /**< debouncing timer */
    struct rt_timer timer2; /**< timer for advanced action evaluation */
    button_action_t last_action;
    button_cfg_t cfg;     /**< button configuration  */
#ifdef BUTTON_SERVICE_ENABLED
    datas_handle_t service_handle;
#endif /* BUTTON_SERVICE_ENABLED */
} button_item_t;


typedef struct
{
    bool init;              /**< if button library is initialized */
    struct rt_semaphore sema;
    button_item_t   buttons[BUTTON_MAX_NUM];
} button_ctx_t;


#ifdef USING_ADC_BUTTON

typedef struct
{
    uint16_t voltage;
    adc_button_handler_t handler;
#ifdef BUTTON_SERVICE_ENABLED
    datas_handle_t service_handle;
#endif /* BUTTON_SERVICE_ENABLED */
} adc_button_cfg_t;

typedef struct
{
    //bool hpsys_pin;
    //int pad;
    int8_t last_btn_idx;
} adc_button_group_dyn_cfg_t;

typedef struct
{
    uint8_t num;
    uint8_t channel;
    adc_button_cfg_t *config;
    button_handler_t handler;
} adc_button_group_cfg_t;
#endif /* USING_ADC_BUTTON */

#ifdef ADC_BUTTON_GROUP1_MAX_NUM
    static void adc_button_group1_handler(int32_t pin, button_action_t button_action);
#endif /* ADC_BUTTON_GROUP1_MAX_NUM */

#ifdef ADC_BUTTON_GROUP2_MAX_NUM
    static void adc_button_group2_handler(int32_t pin, button_action_t button_action);
#endif /* ADC_BUTTON_GROUP2_MAX_NUM */

/** button list */
static button_ctx_t s_button_ctx;

#ifdef ADC_BUTTON_GROUP1_MAX_NUM
static adc_button_cfg_t s_adc_button_group1[ADC_BUTTON_GROUP1_MAX_NUM] =
{
#ifdef ADC_BUTTON_GROUP1_BUTTON1_VOLT
    {ADC_BUTTON_GROUP1_BUTTON1_VOLT, NULL},
#endif /* ADC_BUTTON_GROUP1_BUTTON1_VOLT */

#ifdef ADC_BUTTON_GROUP1_BUTTON2_VOLT
    {ADC_BUTTON_GROUP1_BUTTON2_VOLT, NULL},
#endif /* ADC_BUTTON_GROUP1_BUTTON2_VOLT */

#ifdef ADC_BUTTON_GROUP1_BUTTON3_VOLT
    {ADC_BUTTON_GROUP1_BUTTON3_VOLT, NULL}
#endif /* ADC_BUTTON_GROUP1_BUTTON3_VOLT */
};

#endif /* ADC_BUTTON_GROUP1_MAX_NUM */

#ifdef ADC_BUTTON_GROUP2_MAX_NUM
static adc_button_cfg_t s_adc_button_group2[ADC_BUTTON_GROUP2_MAX_NUM] =
{
#ifdef ADC_BUTTON_GROUP2_BUTTON1_VOLT
    {ADC_BUTTON_GROUP2_BUTTON1_VOLT, NULL}
#endif /* ADC_BUTTON_GROUP1_BUTTON1_VOLT */

#ifdef ADC_BUTTON_GROUP2_BUTTON2_VOLT
    {ADC_BUTTON_GROUP2_BUTTON2_VOLT, NULL}
#endif /* ADC_BUTTON_GROUP1_BUTTON2_VOLT */

#ifdef ADC_BUTTON_GROUP2_BUTTON3_VOLT
    {ADC_BUTTON_GROUP2_BUTTON3_VOLT, NULL}
#endif /* ADC_BUTTON_GROUP1_BUTTON3_VOLT */
};
#endif /* ADC_BUTTON_GROUP2_MAX_NUM */

#ifdef USING_ADC_BUTTON
const adc_button_group_cfg_t s_adc_button_groups[ADC_BUTTON_GROUP_NUM] =
{
#if ADC_BUTTON_GROUP_NUM > 0
    {ADC_BUTTON_GROUP1_MAX_NUM,  ADC_BUTTON_GROUP1_ADC_DEV_CHANNEL, s_adc_button_group1, adc_button_group1_handler},
#endif /* ADC_BUTTON_GROUP_NUM > 0 */

#if ADC_BUTTON_GROUP_NUM > 1
    {ADC_BUTTON_GROUP2_MAX_NUM,  ADC_BUTTON_GROUP2_ADC_DEV_CHANNEL, s_adc_button_group2, adc_button_group2_handler},
#endif /* ADC_BUTTON_GROUP_NUM > 1 */

#if ADC_BUTTON_GROUP_NUM > 2
#error "not supported"
#endif
};

static adc_button_group_dyn_cfg_t s_adc_button_groups_dyn_cfg[ADC_BUTTON_GROUP_NUM];
static rt_device_t s_adc_dev;
#endif /* USING_ADC_BUTTON */


inline static void get_group_pin_idx(int pin, uint8_t *group_idx, uint8_t *pin_idx)
{
    RT_ASSERT(group_idx && pin_idx);
    *group_idx = pin / GROUP_PIN_NUM;
    *pin_idx = pin - *group_idx * GROUP_PIN_NUM;
    RT_ASSERT(*group_idx < MAX_GROUP_NUM);
}

static bool is_pressed(int pin_state, button_active_state_t active_state)
{
    bool pressed;

    if (pin_state ^ (active_state == BUTTON_ACTIVE_HIGH))
    {
        pressed = false;
    }
    else
    {
        pressed = true;
    }
    return pressed;
}


#ifdef USING_ADC_BUTTON

static int8_t button_find_adc_button_id(const adc_button_group_cfg_t *adc_btn_group_cfg,
                                        int32_t pin)
{
    GPIO_TypeDef *gpio_instance;
    int hcpu;
    int pad;
    pin_function func;
    int gpiox_pin;
    rt_err_t r;
    rt_adc_cmd_read_arg_t read_arg;
    uint32_t i;
    adc_button_cfg_t *adc_btn_cfg;

    adc_btn_cfg = adc_btn_group_cfg->config;
    gpio_instance = GET_GPIO_INSTANCE(pin);
    gpiox_pin = GET_GPIOx_PIN(pin);
    if (hwp_gpio1 == gpio_instance)
    {
        hcpu = 1;
        pad = PAD_PA00 + gpiox_pin;
        func = GPIO_A0 + gpiox_pin;
    }
    else if (hwp_gpio2 == gpio_instance)
    {
        hcpu = 0;
        pad = PAD_PB00 + GET_GPIOx_PIN(pin);
        func = GPIO_B0 + gpiox_pin;
    }
    else
    {
        RT_ASSERT(0);
    }

    read_arg.channel = adc_btn_group_cfg->channel;

    //rt_kprintf("start_adc\n");

    rt_pin_irq_enable(pin, false);

    HAL_PIN_Set_Analog(pad, hcpu);

    r = rt_adc_enable((rt_adc_device_t)s_adc_dev, read_arg.channel);
    RT_ASSERT(RT_EOK == r);
    HAL_Delay_us(100);
    r = rt_device_control(s_adc_dev, RT_ADC_CMD_READ, &read_arg.channel);
    RT_ASSERT(RT_EOK == r);

    rt_adc_disable((rt_adc_device_t)s_adc_dev, read_arg.channel);

    HAL_PIN_Set(pad, func, PIN_NOPULL, hcpu);
    HAL_PIN_SetMode(pad, hcpu, PIN_DIGITAL_IO_NORMAL);

    rt_pin_irq_enable(pin, true);

    /* convert 0.1mV to mV */
    read_arg.value /= 10;
    for (i = 0; i < adc_btn_group_cfg->num; i++)
    {
        if ((read_arg.value >= (adc_btn_cfg[i].voltage - BUTTON_VOLT_VALID_RANGE))
                && (read_arg.value <= (adc_btn_cfg[i].voltage + BUTTON_VOLT_VALID_RANGE)))
        {
            break;
        }
    }

    //rt_kprintf("volt:%d\n", read_arg.value);

    if (i >= adc_btn_group_cfg->num)
    {
        return -1;
    }
    else
    {
        return i;
    }
}

#ifdef BUTTON_SERVICE_ENABLED
static void send_adc_button_action(adc_button_cfg_t *adc_btn_cfg, button_action_t action)
{
    RT_ASSERT(adc_btn_cfg);
    datas_data_ready(adc_btn_cfg->service_handle, sizeof(uint8_t *), (uint8_t *)action);
}
#endif /* BUTTON_SERVICE_ENABLED */

static void adc_button_group_handler(uint8_t group_idx, int32_t pin, button_action_t button_action)
{
    const adc_button_group_cfg_t *adc_btn_group_cfg;
    adc_button_cfg_t *adc_btn_cfg;
    adc_button_group_dyn_cfg_t *dyn_cfg;
    int8_t btn_id;

    RT_ASSERT(group_idx < ADC_BUTTON_GROUP_NUM);

    dyn_cfg = &s_adc_button_groups_dyn_cfg[group_idx];
    adc_btn_group_cfg = &s_adc_button_groups[group_idx];
    adc_btn_cfg = adc_btn_group_cfg->config;

    if (BUTTON_PRESSED == button_action)
    {
        btn_id = button_find_adc_button_id(adc_btn_group_cfg, pin);
        dyn_cfg->last_btn_idx = btn_id;
    }
    else
    {
        //RT_ASSERT((dyn_cfg->last_btn_idx >= 0) && (dyn_cfg->last_btn_idx < adc_btn_group_cfg->num));
        btn_id = dyn_cfg->last_btn_idx;
    }

    if ((btn_id >= 0) && (btn_id < adc_btn_group_cfg->num)
            && (adc_btn_cfg[btn_id].handler))
    {
        adc_btn_cfg[btn_id].handler(group_idx, btn_id, button_action);
#ifdef BUTTON_SERVICE_ENABLED
        send_adc_button_action(&adc_btn_cfg[btn_id], button_action);
#endif /* BUTTON_SERVICE_ENABLED */
    }
    else
    {
        rt_kprintf("Unknown pin:%d,%d,%d\n", group_idx, pin, button_action);
    }
}
#endif /* USING_ADC_BUTTON */


#ifdef ADC_BUTTON_GROUP1_MAX_NUM
static void adc_button_group1_handler(int32_t pin, button_action_t button_action)
{
    adc_button_group_handler(0, pin, button_action);
}
#endif

#ifdef ADC_BUTTON_GROUP2_MAX_NUM
static void adc_button_group2_handler(int32_t pin, button_action_t button_action)
{
    adc_button_group_handler(1, pin, button_action);
}
#endif



#ifdef BUTTON_SERVICE_ENABLED
static void send_button_action(button_item_t *button, button_action_t action)
{
    RT_ASSERT(button);
    datas_data_ready(button->service_handle, sizeof(uint8_t *), (uint8_t *)action);
}
#endif /* BUTTON_SERVICE_ENABLED */

static void detection_timeout_handler(void *parameter)
{
    button_item_t *button;
    button_action_t action;
    rt_base_t mask;
    uint32_t button_id;
    int curr_state;
    rt_err_t err;

    button_id = (uint32_t)parameter;
    RT_ASSERT(button_id < BUTTON_MAX_NUM);
    button = &s_button_ctx.buttons[button_id];
    if (!button->enabled || !button->valid)
    {
        return;
    }

    curr_state = rt_pin_read(button->cfg.pin);
    mask = rt_hw_interrupt_disable();
    if (button->last_state == curr_state)
    {
        if (button->debounce_cnt > 0)
        {
            button->debounce_cnt--;
        }
    }
    else
    {
        button->debounce_cnt = button->cfg.debounce_time;
        button->last_state = curr_state;
    }
    rt_hw_interrupt_enable(mask);

    if (button->debounce_cnt > 0)
    {
        err = rt_timer_start(&button->timer);
        SF_ASSERT(RT_EOK == err);
    }
    else
    {
        rt_timer_stop(&button->timer2);
        if (is_pressed(curr_state, button->cfg.active_state))
        {
            action = BUTTON_PRESSED;
            err = rt_timer_start(&button->timer2);
            SF_ASSERT(RT_EOK == err);
        }
        else
        {
            action = BUTTON_RELEASED;
        }
        SF_ASSERT(button->cfg.button_handler);
        if ((BUTTON_PRESSED == button->last_action) && (BUTTON_RELEASED == action))
        {
            button->cfg.button_handler(button->cfg.pin, BUTTON_CLICKED);
#ifdef BUTTON_SERVICE_ENABLED
            send_button_action(button, BUTTON_CLICKED);
#endif /* BUTTON_SERVICE_ENABLED */

        }
        button->last_action = action;
        button->cfg.button_handler(button->cfg.pin, action);
#ifdef BUTTON_SERVICE_ENABLED
        send_button_action(button, action);
#endif /* BUTTON_SERVICE_ENABLED */
    }
}



static void timer2_timeout_handler(void *parameter)
{
    uint32_t button_id;
    button_item_t *button;

    button_id = (uint32_t)parameter;
    RT_ASSERT(button_id < BUTTON_MAX_NUM);
    button = &s_button_ctx.buttons[button_id];
    if (!button->enabled || !button->valid)
    {
        return;
    }

    if (BUTTON_PRESSED == button->last_action)
    {
        button->last_action = BUTTON_LONG_PRESSED;
        SF_ASSERT(button->cfg.button_handler);
        button->cfg.button_handler(button->cfg.pin, button->last_action);
#ifdef BUTTON_SERVICE_ENABLED
        send_button_action(button, button->last_action);
#endif /* BUTTON_SERVICE_ENABLED */
    }
}

static void pin_event_handler(void *args)
{
    int32_t pin = (int32_t)args;
    rt_err_t err;
    rt_base_t mask;
    button_item_t *button;
    uint32_t i;

    for (i = 0; i < BUTTON_MAX_NUM; i++)
    {
        if (!s_button_ctx.buttons[i].enabled)
        {
            continue;
        }
        if (pin == s_button_ctx.buttons[i].cfg.pin)
        {
            break;
        }
    }
    if (i < BUTTON_MAX_NUM)
    {
        button = &s_button_ctx.buttons[i];
        if (0 == button->debounce_cnt)
        {
            mask = rt_hw_interrupt_disable();
            button->debounce_cnt = button->cfg.debounce_time;
            rt_hw_interrupt_enable(mask);

            button->last_state = rt_pin_read(pin);
            err = rt_timer_start(&button->timer);
            SF_ASSERT(RT_EOK == err);
        }
    }
}


#ifdef BSP_USING_PM
static int button_pm_suspend(const struct rt_device *device, uint8_t mode)
{
    int r = RT_EOK;

    return r;
}

void button_pm_resume(const struct rt_device *device, uint8_t mode)
{
    button_ctx_t *ctx = &s_button_ctx;
    uint32_t i;
    button_cfg_t *cfg;
    rt_err_t err;

    if (PM_SLEEP_MODE_STANDBY != mode)
    {
        return;
    }

    for (i = 0; i < BUTTON_MAX_NUM; i++)
    {
        if (ctx->buttons[i].valid)
        {
            cfg = &ctx->buttons[i].cfg;
            rt_pin_mode(cfg->pin, cfg->mode);
            err = rt_pin_attach_irq(cfg->pin, PIN_IRQ_MODE_RISING_FALLING, pin_event_handler, (void *)cfg->pin);
            SF_ASSERT(err == RT_EOK);
            if (ctx->buttons[i].enabled)
            {
                button_enable(i);
            }
        }
    }

    return ;
}


const struct rt_device_pm_ops button_pm_op =
{
    .suspend = button_pm_suspend,
    .resume = button_pm_resume,
};

#endif /* BSP_USING_PM */


#ifdef BUTTON_SERVICE_ENABLED
static int32_t button_service_msg_handler(datas_handle_t service, data_msg_t *msg)
{
    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        break;
    }
    case MSG_SERVICE_CONFIG_REQ:
    {
        break;
    }
    case MSG_SERVICE_DATA_RDY_IND:
    {
        data_rdy_ind_t *data_ind = (data_rdy_ind_t *)(data_service_get_msg_body(msg));
        button_action_t action;

        RT_ASSERT(data_ind);

        action = (button_action_t)data_ind->data;
        datas_push_data_to_client(service, sizeof(action), &action);
        break;
    }
    case MSG_SERVICE_BUTTON_IRQ_TRIGGER_REQ:
    {
        for (uint32_t i = 0; i < BUTTON_MAX_NUM; i++)
        {
            if (service == s_button_ctx.buttons[i].service_handle)
            {
                button_irq_trigger(s_button_ctx.buttons[i].cfg.pin);
                break;
            }
        }
        datas_send_response(service, msg, 0);
        break;
    }
    default:
    {
        RT_ASSERT(0);
    }
    }

    return 0;
}


static data_service_config_t button_service_cb =
{
    .max_client_num = BUTTON_SERVICE_MAX_CLIENT_NUM,
    .queue = RT_NULL,
    .data_filter = NULL,
    .msg_handler = button_service_msg_handler,
};

#endif /* BUTTON_SERVICE_ENABLED */

int32_t button_init(button_cfg_t *cfg)
{
    rt_err_t err;
    uint32_t i;
    int32_t button_id;
    button_ctx_t *ctx = &s_button_ctx;
    button_item_t *button;
    uint32_t mask;
#ifdef BUTTON_SERVICE_ENABLED
    char btn_service_name[5];
#endif /* BUTTON_SERVICE_ENABLED */

    mask = rt_hw_interrupt_disable();
    if (!ctx->init)
    {
        ctx->init = true;
        rt_sem_init(&ctx->sema, "btn", 1, RT_IPC_FLAG_FIFO);
#ifdef BSP_USING_PM
        rt_pm_device_register(NULL, &button_pm_op);
#endif /* BSP_USING_PM */
        rt_hw_interrupt_enable(mask);

    }
    else
    {
        rt_hw_interrupt_enable(mask);
    }

    err = rt_sem_take(&ctx->sema, RT_WAITING_FOREVER);
    SF_ASSERT(err == RT_EOK);

    if (NULL == cfg)
    {
        button_id = -SF_EINVAL;
        goto __END;
    }

    if ((cfg->pin > MAX_PIN)
            || (NULL == cfg->button_handler))
    {
        button_id = -SF_EINVAL;
        goto __END;
    }

    button_id = -1;
    for (i = 0; i < BUTTON_MAX_NUM; i++)
    {
        /* record the first free position */
        if (button_id < 0)
        {
            if (!ctx->buttons[i].valid)
            {
                button_id = i;
            }
        }
        if (ctx->buttons[i].valid && (ctx->buttons[i].cfg.pin == cfg->pin))
        {
            button_id = -SF_EBUSY;
            goto __END;
        }
    }

    if (button_id < 0)
    {
        button_id = -SF_ERR;
        goto __END;
    }

    button = &ctx->buttons[button_id];
    button->valid = true;
    button->last_action = BUTTON_RELEASED;
    button->enabled = false;
    button->debounce_cnt = 0;
    memcpy(&button->cfg, cfg, sizeof(button->cfg));
    /* TODO: use fixed value for now */
    button->cfg.debounce_time = 20;
    rt_timer_init(&button->timer, "btn", detection_timeout_handler, (void *)button_id,
                  rt_tick_from_millisecond(BUTTON_DETECTION_DELAY), RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    rt_timer_init(&button->timer2, "btn_tm2", timer2_timeout_handler, (void *)button_id,
                  rt_tick_from_millisecond(BUTTON_ADV_ACTION_CHECK_DELAY), RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);

#ifdef BUTTON_SERVICE_ENABLED
    rt_snprintf(btn_service_name, sizeof(btn_service_name), "btn%d", button_id);
    button->service_handle = datas_register(btn_service_name, &button_service_cb);
#endif /* BUTTON_SERVICE_ENABLED */

    rt_pin_mode(cfg->pin, cfg->mode);
    err = rt_pin_attach_irq(cfg->pin, PIN_IRQ_MODE_RISING_FALLING, pin_event_handler, (void *)cfg->pin);
    SF_ASSERT(err == RT_EOK);

__END:
    err = rt_sem_release(&ctx->sema);
    SF_ASSERT(err == RT_EOK);

    return button_id;
}

sf_err_t button_enable(int32_t id)
{
    button_item_t *button;
    sf_err_t err;

    if ((id < 0) || (id >= BUTTON_MAX_NUM))
    {
        return -SF_ERR;
    }

    button = &s_button_ctx.buttons[id];
    if (!button->valid)
    {
        return -SF_ERR;
    }

    button->debounce_cnt = 0;
    if (RT_EOK == rt_pin_irq_enable(button->cfg.pin, true))
    {
        button->enabled = true;
        err = SF_EOK;
    }
    else
    {
        err = -SF_ERR;
    }

    return err;
}


sf_err_t button_disable(int32_t id)
{
    button_item_t *button;
    sf_err_t err;

    if ((id < 0) || (id >= BUTTON_MAX_NUM))
    {
        return -SF_ERR;
    }

    button = &s_button_ctx.buttons[id];
    if (!button->valid)
    {
        return -SF_ERR;
    }

    if (RT_EOK == rt_pin_irq_enable(button->cfg.pin, false))
    {
        button->enabled = false;
        err = SF_EOK;
    }
    else
    {
        err = -SF_ERR;
    }

    return err;
}


bool button_is_pressed(int32_t id)
{
    button_item_t *button;
    bool pressed;
    int pin_state;

    SF_ASSERT((id >= 0) && (id < BUTTON_MAX_NUM));

    button = &s_button_ctx.buttons[id];
    SF_ASSERT(button->valid);

    pin_state = rt_pin_read(button->cfg.pin);

    return is_pressed(pin_state, button->cfg.active_state);
}

void button_irq_trigger(int32_t pin)
{
    pin_event_handler((void *)pin);
}

sf_err_t button_update_handler(int32_t id, button_handler_t new_handler)
{
    button_item_t *button;

    if ((id < 0) || (id >= BUTTON_MAX_NUM))
    {
        return -SF_ERR;
    }

    button = &s_button_ctx.buttons[id];
    if (!button->valid)
    {
        return -SF_ERR;
    }

    button->cfg.button_handler = new_handler;

    return SF_EOK;
}

#ifdef USING_ADC_BUTTON
int32_t button_bind_adc_button(int32_t id, int8_t adc_button_group_id, uint8_t handler_num,
                               adc_button_handler_t *handler)
{
    button_item_t *button;
    sf_err_t err;
    const adc_button_group_cfg_t *adc_btn_group_cfg;
    adc_button_group_dyn_cfg_t *adc_btn_group_dyn_cfg;
    adc_button_cfg_t *adc_btn_cfg;
    uint32_t i;
#ifdef BUTTON_SERVICE_ENABLED
    char btn_service_name[10];
#endif /* BUTTON_SERVICE_ENABLED */


    if ((id < 0) || (id >= BUTTON_MAX_NUM))
    {
        return -SF_ERR;
    }

    button = &s_button_ctx.buttons[id];
    if (!button->valid)
    {
        return -SF_ERR;
    }

    if (adc_button_group_id >= ADC_BUTTON_GROUP_NUM)
    {
        return -SF_ERR;
    }

    if (adc_button_group_id >= 0)
    {
        adc_btn_group_cfg = &s_adc_button_groups[adc_button_group_id];
        adc_btn_group_dyn_cfg = &s_adc_button_groups_dyn_cfg[adc_button_group_id];
        adc_btn_group_dyn_cfg->last_btn_idx = -1;

        if (handler_num > adc_btn_group_cfg->num)
        {
            return -SF_EFULL;
        }
        adc_btn_cfg = adc_btn_group_cfg->config;
        for (i = 0; i < handler_num; i++)
        {
            adc_btn_cfg[i].handler = handler[i];

#ifdef BUTTON_SERVICE_ENABLED
            rt_snprintf(btn_service_name, sizeof(btn_service_name), "btn%d-%d", adc_button_group_id, i);
            adc_btn_cfg[i].service_handle = datas_register(btn_service_name, &button_service_cb);
            RT_ASSERT(adc_btn_cfg[i].service_handle);
#endif /* BUTTON_SERVICE_ENABLED */
        }
        button->cfg.button_handler = adc_btn_group_cfg->handler;
    }

    if (!s_adc_dev)
    {
        s_adc_dev = rt_device_find(ADC_BUTTON_ADC_DEV_NAME);
        RT_ASSERT(s_adc_dev);
    }

    return SF_EOK;
}

#endif /* USING_ADC_BUTTON */

