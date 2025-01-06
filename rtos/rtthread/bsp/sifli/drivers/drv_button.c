/**
  ******************************************************************************
  * @file   drv_button.c
  * @author Sifli software development team
  * @brief Button BSP driver
  * @{
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
#include <string.h>
#include "drv_button.h"
#include "board.h"

#define LOG_TAG                      "drv.btn"
#include "drv_log.h"

/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_button Button
  * @brief Button BSP driver
  * @{
  */


typedef struct
{
    GPT_TypeDef *instance;
    IRQn_Type irqn;
    uint8_t opened;
} drv_button_cfg_t;


/*******************************************************************
 *                          Variable Declaration
 *******************************************************************/
static rt_slist_t g_buttons;

static GPT_HandleTypeDef button_handler;

static struct rt_device g_button_device;

static drv_button_cfg_t drv_button_cfg =
{
#ifdef BSP_USING_GPT2_BUTTON
    .instance = hwp_gptim2,
    .irqn = GPTIM2_IRQn,
#elif defined(BSP_USING_GPT3_BUTTON)
    .instance = hwp_gptim3,
    .irqn = GPTIM3_IRQn,
#elif defined(BSP_USING_GPT5_BUTTON)
    .instance = hwp_gptim5,
    .irqn = GPTIM5_IRQn,
#else
#error "wrong config"
#endif
};



/**
  ************************************************************
  * @brief   Create a Button
  * @param[in]   name button name
  * @param[in]   btn button structure
  * @param[in]   read_btn_level Button trigger level reading function,
  *                 Return the level of the rt_uint8_t type by yourself
  * @param[in]   btn_trigger_level Button trigger level
  ***********************************************************/
void button_create(const char *name,
                   button_t *btn,
                   rt_uint8_t(*read_btn_level)(void),
                   rt_uint8_t btn_trigger_level)
{
    if (btn == RT_NULL)
        LOG_E("struct button is RT_NULL!");

    memset(btn, 0, sizeof(struct button));      //Clear structure information
    strcpy(btn->name, name);      //button name

    btn->btn_state = NONE_TRIGGER;                     //Button status
    btn->btn_last_state = NONE_TRIGGER;                //Button last status
    btn->btn_trigger_evt = NONE_TRIGGER;               //Button trigger event
    btn->read_btn_level = read_btn_level;              //Button trigger level reading function
    btn->btn_trigger_level = btn_trigger_level;        //Button trigger level
    btn->btn_last_level = btn->read_btn_level();       //Button current level

    LOG_I("button create success!");
    rt_slist_append((rt_slist_t *)&g_buttons, (rt_slist_t *)btn);
}

/**
  ***********************************************************
  * @brief   burron trigger events are attach to callback function
  * @param[in]   btn button structure
  * @param[in]   callback   Callback handler after the button is triggered.Need user implementation
  ***********************************************************/
void button_regcbk(button_t *btn, button_cbk callback)
{
    if (btn == RT_NULL)
        RT_DEBUG_LOG(RT_DEBUG_THREAD, ("struct button is RT_NULL!"));
    else
        btn->cbk_func = callback;
}

/**
  ***********************************************************
  * @brief   Delete an already created button
  * @param[in]   btn button structure
  ***********************************************************/
void button_delete(button_t *btn)
{
    rt_slist_remove((rt_slist_t *)&g_buttons, (rt_slist_t *)btn);
}

/**
  ***********************************************************
  * @brief   button cycle processing function
  * @param[in]   btn button structure
  * @note    This function must be called in a certain period. The recommended period is 20~50ms.
  ***********************************************************/
static void button_cycle_process(button_t *btn)
{
    /* Get the current button level */
    rt_uint8_t current_level = (rt_uint8_t)btn->read_btn_level();

    /* Button level changes, debounce */
    if ((current_level != btn->btn_last_level) && (++(btn->btn_debounce) >= BUTTON_DEBOUNCE_TIME))
    {
        btn->btn_last_level = current_level;    /* Update current button level */
        btn->btn_debounce = 0;                  /* button is pressed */
        /* If the button is not pressed, change the button state to press (first press / double trigger) */
        if ((btn->btn_state == NONE_TRIGGER) || (btn->btn_state == BUTTON_DOUBLE))
            btn->btn_state = BUTTON_DOWM;
        else if (btn->btn_state == BUTTON_DOWM) //free button
            btn->btn_state = BUTTON_UP;
    }

    switch (btn->btn_state)
    {
    case BUTTON_DOWM :
    {
        if (btn->btn_last_level == btn->btn_trigger_level)
        {
#ifdef CONTINUOS_TRIGGER  /* Support continuous triggering */
            if (++(btn->btn_cycle) >= BUTTON_CONTINUOS_CYCLE)
            {
                btn->btn_cycle = 0;
                btn->btn_trigger_evt = BUTTON_CONTINUOS;
                TRIGGER_CB(BUTTON_CONTINUOS);  /* continuous triggering */
            }
#else
            btn->btn_trigger_evt = BUTTON_DOWM;

            /* Update the trigger event before releasing the button as long press */
            if (++(btn->btn_long_time) >= BUTTON_LONG_TIME)
            {
#ifdef LONG_FREE_TRIGGER
                btn->btn_trigger_evt = BUTTON_LONG;
#else
                /* Continuous triggering of long press cycles */
                if (++(btn->btn_cycle) >= BUTTON_LONG_CYCLE)
                {
                    btn->btn_cycle = 0;
                    btn->btn_trigger_evt = BUTTON_LONG;
                    TRIGGER_CB(BUTTON_LONG);        /* long triggering */
                }
#endif
                if (btn->btn_long_time == 0xFF)     /* Update time overflow */
                    btn->btn_long_time = BUTTON_LONG_TIME;
            }
#endif
        }
        break;
    }
    case BUTTON_UP :
    {
        if (btn->btn_trigger_evt == BUTTON_DOWM)
        {
            if ((btn->btn_count <= BUTTON_DOUBLE_TIME) && (btn->btn_last_state == BUTTON_DOUBLE)) /* double click */
            {
                btn->btn_trigger_evt = BUTTON_DOUBLE;
                TRIGGER_CB(BUTTON_DOUBLE);
                btn->btn_state = NONE_TRIGGER;
                btn->btn_last_state = NONE_TRIGGER;
            }
            else
            {
                btn->btn_count = 0;
                btn->btn_long_time = 0; /* Detection long press failed, clear 0 */
#ifndef SINGLE_AND_DOUBLE_TRIGGER
                TRIGGER_CB(BUTTON_DOWM);    /* click */
#endif
                btn->btn_state = BUTTON_DOUBLE;
                btn->btn_last_state = BUTTON_DOUBLE;
            }
        }
        else if (btn->btn_trigger_evt == BUTTON_LONG)
        {
#ifdef LONG_FREE_TRIGGER
            TRIGGER_CB(BUTTON_LONG);    /* Long press */
#else
            TRIGGER_CB(BUTTON_LONG_FREE); /* Long press free */
#endif
            btn->btn_long_time = 0;
            btn->btn_state = NONE_TRIGGER;
            btn->btn_last_state = BUTTON_LONG;
        }
#ifdef CONTINUOS_TRIGGER
        else if (btn->btn_trigger_evt == BUTTON_CONTINUOS) /* Press continuously */
        {
            btn->btn_long_time = 0;
            TRIGGER_CB(BUTTON_CONTINUOS_FREE);    /* Press continuously free */
            btn->btn_state = NONE_TRIGGER;
            btn->btn_last_state = BUTTON_CONTINUOS;
        }
#endif
        break;
    }

    case BUTTON_DOUBLE :
    {
        /* Update time */
        btn->btn_count++;
        if (btn->btn_count >= BUTTON_DOUBLE_TIME)
        {
            btn->btn_state = NONE_TRIGGER;
            btn->btn_last_state = NONE_TRIGGER;
        }
#ifdef SINGLE_AND_DOUBLE_TRIGGER
        if ((btn->btn_count >= BUTTON_DOUBLE_TIME) && (btn->btn_last_state != BUTTON_DOWM))
        {
            btn->btn_count = 0;
            TRIGGER_CB(BUTTON_DOWM);
            btn->btn_state = NONE_TRIGGER;
            btn->btn_last_state = BUTTON_DOWM;
        }
#endif
        break;
    }
    default :
        break;
    }
}

/**
  ************************************************************
  * @brief   Traversing the way to scan the button without losing each button
  * @note    This function is called periodically, it is recommended to call 20-50ms once.
  ***********************************************************/
void button_process(void)
{
    rt_slist_t *btn;

    rt_slist_for_each(btn, (&g_buttons))
    button_cycle_process((button_t *)btn);
}

static uint32_t g_gpt_press_key = 0;
static button_event g_gpt_state = BUTTON_UP;


__ROM_USED void HAL_GPT_IC_CaptureCallback(GPT_HandleTypeDef *htim)
{
    //LOG_I("HAL_GPT_IC_CaptureCallback %x, ch:%d\n", htim->Instance, htim->Channel);

    g_gpt_state = BUTTON_DOWM;
    switch (htim->Channel)
    {
    case HAL_GPT_ACTIVE_CHANNEL_1:
        g_gpt_press_key = 1;
        break;

    case HAL_GPT_ACTIVE_CHANNEL_2:
        g_gpt_press_key = 2;  // for evb_z0
        break;

    case HAL_GPT_ACTIVE_CHANNEL_3:
        g_gpt_press_key = 2;  // for fpga_a0
        break;

    case HAL_GPT_ACTIVE_CHANNEL_4:
        g_gpt_press_key = 4;
        break;
    default:
        g_gpt_press_key = 0;
        g_gpt_state = BUTTON_UP;
        break;
    }



}
__ROM_USED void HAL_GPT_IC_MspInit(GPT_HandleTypeDef *htim)
{
    UNUSED(htim);

}
static rt_err_t gpt_button_init(GPT_HandleTypeDef *tim)
{
    rt_err_t result = RT_EOK;
    GPT_IC_InitTypeDef ic_config = {0};
    GPT_ClockConfigTypeDef clock_config = {0};

    //RT_ASSERT(device != RT_NULL);

    //tim = hwp_gptim3; //(GPT_HandleTypeDef *)&device->tim_handle;

    //HAL_GPT_Base_Stop(tim);


    /* configure the timer to pwm mode */
    tim->Init.Prescaler = 4799;  //100us
    tim->Init.CounterMode = GPT_COUNTERMODE_UP;
    tim->Init.Period = 99;

    if (HAL_GPT_IC_Init(tim) != HAL_OK)
    {
        LOG_E("gpt_button_init time base init failed");
        result = -RT_ERROR;
        goto __exit;
    }

    ic_config.ICFilter = 0xc;
    ic_config.ICPrescaler = GPT_ICPSC_DIV1;
    ic_config.ICPolarity = 0;
    ic_config.ICSelection = GPT_ICSELECTION_DIRECTTI;
    HAL_GPT_IC_ConfigChannel(tim, &ic_config, GPT_CHANNEL_1);
    HAL_GPT_IC_ConfigChannel(tim, &ic_config, GPT_CHANNEL_2);
    HAL_GPT_IC_ConfigChannel(tim, &ic_config, GPT_CHANNEL_3);
    HAL_GPT_IC_ConfigChannel(tim, &ic_config, GPT_CHANNEL_4);

    /*
        clock_config.ClockSource = GPT_CLOCKSOURCE_INTERNAL;
        if (HAL_GPT_ConfigClockSource(tim, &clock_config) != HAL_OK)
        {
            LOG_E("%s clock init failed", device->name);
            result = -RT_ERROR;
            goto __exit;
        }

    */

    /* pwm pin configuration */
    //HAL_GPT_MspPostInit(tim);

    /* enable update request source */
    //__HAL_GPT_URS_ENABLE(tim);

    //HAL_GPT_Base_Start_IT(tim);

    // HAL_GPT_IC_Start
    HAL_GPT_IC_Start_IT(tim, GPT_CHANNEL_1);
    HAL_GPT_IC_Start_IT(tim, GPT_CHANNEL_2);
    HAL_GPT_IC_Start_IT(tim, GPT_CHANNEL_3);
    HAL_GPT_IC_Start_IT(tim, GPT_CHANNEL_4);
    //HAL_GPT_Base_Start(tim);

    HAL_NVIC_SetPriority(drv_button_cfg.irqn, 3, 0);
    HAL_NVIC_EnableIRQ(drv_button_cfg.irqn);


__exit:
    return result;
}


/** @defgroup button_device button device functions registered to OS
  * @ingroup drv_button
  * @{
 */


/**
 * @brief initialize button device
 * @param[in] dev button device to initialize
 * @retval RT_EOK if successful.
*/
static rt_err_t bf0_button_init(struct rt_device *dev)
{
    int i = 0;
    int result = RT_EOK;

    button_handler.Instance = drv_button_cfg.instance;
    drv_button_cfg.opened = 0;
    result = gpt_button_init(&button_handler);

__exit:
    if (RT_EOK == result)
        LOG_I("bf0_button_init success\n");
    else
        LOG_E("bf0_button_init failed %d\n", result);
    return result;
}


/**
 * @brief open button device
 * @param[in] dev button device to open
 * @param[in] oflag Flags for opening
 * @retval RT_EOK if successful.
*/
static rt_err_t rt_button_open(struct rt_device *dev, rt_uint16_t oflag)
{
    rt_err_t result = RT_EOK;
    /*
        rt_button_t *timer;

        timer = (rt_button_t *)dev;
        if (timer->ops->control != RT_NULL)
        {
            timer->ops->control(timer, button_CTRL_FREQ_SET, &timer->freq);
        }
        else
        {
            result = -RT_ENOSYS;
        }
    */
    drv_button_cfg.opened++;
    return result;
}

/**
 * @brief close button device
 * @param[in] dev button device to close
 * @retval RT_EOK if successful.
*/
static rt_err_t rt_button_close(struct rt_device *dev)
{
    rt_err_t result = RT_EOK;
    /*
        rt_button_t *timer;

        timer = (rt_button_t*)dev;
        if (timer->ops->init != RT_NULL)
        {
            timer->ops->init(timer, 0);
        }
        else
        {
            result = -RT_ENOSYS;
        }

        dev->flag &= ~RT_DEVICE_FLAG_ACTIVATED;
        dev->rx_indicate = RT_NULL;
    */
    if (drv_button_cfg.opened)
        drv_button_cfg.opened--;
    return result;
}

/**
 * @brief Read data from button device
 * @param[in] dev button device to read
 * @param[in] pos position to read
 * @param[in,out] buffer read buffer
 * @param[in] size size to read.
 * @retval size read
*/
static rt_size_t rt_button_read(struct rt_device *dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_uint32_t *bufptr = (rt_uint32_t *) buffer;


    if (size < 4) return 0;

    *bufptr = g_gpt_press_key;
    *(bufptr + 1) = (BUTTON_UP == g_gpt_state) ? 0 : 1;

    if (BUTTON_DOWM == g_gpt_state)
    {
        g_gpt_state = BUTTON_UP;
    }
    else
    {
    }

    return size;
}


static rt_err_t rt_button_ioctl(struct rt_device *dev, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_SUSPEND:
        break;
    case RT_DEVICE_CTRL_RESUME:
        /* resume device */
#ifdef RT_USING_PM
        if (drv_button_cfg.opened)
            gpt_button_init(&button_handler);
#endif
        break;
    }
    return result;
}

#ifdef RT_USING_DEVICE_OPS
const rt_device_ops button_ops =
{
    bf0_button_init,
    rt_button_open,
    rt_button_close,
    rt_button_read,
    NULL,
    NULL,
    rt_button_ioctl,
};
#endif

/// @} button_device

static int button_init(void)
{
    rt_err_t err = RT_EOK;

    rt_device_t device;

    device = &g_button_device;

    device->type        = RT_Device_Class_Char;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    device->ops         = &button_ops;
#else
    device->init        = bf0_button_init;
    device->open        = rt_button_open;
    device->close       = rt_button_close;
    device->read        = rt_button_read;
    device->write       = NULL;
    device->control     = rt_button_ioctl;
#endif
    device->user_data   = NULL;

    err = rt_device_register(device, "keypad", RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_STANDALONE);

    if (err != RT_EOK)
    {
        LOG_E("button_init failed, err=%d\n", err);
    }
    return 0;
}
INIT_DEVICE_EXPORT(button_init);

//#define DRV_TEST
#ifdef DRV_TEST

static button_t button1;
static int stop_check;
static rt_device_t dev_pin;
static rt_base_t pin_num;

char button_event_name[][8] =
{
    "DOWM",
    "UP",
    "DOUBLE",
    "LONG",
    "LONG_FR",
    "CONT",
    "CONT_FR",
    "NONE"
};

rt_uint8_t read_button1()
{
    return rt_pin_read(pin_num);
}

void button1_callback(struct button *btn, button_event event)
{
    LOG_I("Button %s, event %s\n", btn->name, button_event_name[event]);
    if (event == BUTTON_DOUBLE)
        stop_check = 1;
}

int cmd_button(int argc, char *argv[])
{
    if (strcmp(argv[1], "create") == 0)
    {
        if (argc > 3)
        {
            pin_num = atoi(argv[3]);
            button_create(argv[2], &button1, read_button1, 0);
            button_regcbk(&button1, button1_callback);
        }
    }
    else if (strcmp(argv[1], "delete") == 0)
        button_delete(&button1);
    else if (strcmp(argv[1], "check") == 0)
    {
        LOG_I("Check buttons, double click to exit loop\n");
        while (stop_check == 0)
        {
            button_process();     //Need to call the button handler function periodically
            rt_thread_mdelay(20);
        }
        LOG_I("Button check done\n");
    }
    return 0;
}
FINSH_FUNCTION_EXPORT(cmd_button, Test button functions.)

#endif

/// @} drv_button
/// @} bsp_driver
/// @} file

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
