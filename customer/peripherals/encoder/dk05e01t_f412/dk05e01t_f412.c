/**
  ******************************************************************************
  * @file   dk05e01t_f412.c
  * @author Sifli software development team
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
#include "board.h"
#include "string.h"


/* Define -------------------------------------------------------------------*/
#define DBG_LEVEL          DBG_ERROR  //DBG_LOG // 
#define LOG_TAG              "drv.dk05"
#include <drv_log.h>

enum
{
    CLOCK_WISE,
    COUNTER_CLOCK_WISE,
};


static struct rt_device this_device;
static int32_t btn_id;
static int16_t diff;
rt_timer_t p_timer;

#ifndef ENCODE_INPUT_PIN_A
    #ifdef BSP_USING_BOARD_EC_LB557XXX
        #define ENCODE_INPUT_PIN_A 45
    #elif defined(BSP_USING_BOARD_EH_SS6600XXX)
        #define ENCODE_INPUT_PIN_A 121
    #elif defined(BSP_USING_BOARD_EC_LB555_WATCH)
        #define ENCODE_INPUT_PIN_A 115
    #else
        #define ENCODE_INPUT_PIN_A 78
    #endif
#endif

#ifndef ENCODE_INPUT_PIN_B
    #ifdef BSP_USING_BOARD_EC_LB557XXX
        #define ENCODE_INPUT_PIN_B 41
    #elif defined(BSP_USING_BOARD_EH_SS6600XXX)
        #define ENCODE_INPUT_PIN_B 125
    #elif defined(BSP_USING_BOARD_EC_LB555_WATCH)
        #define ENCODE_INPUT_PIN_B 118
    #else
        #define ENCODE_INPUT_PIN_B 77
    #endif
#endif


#if defined(BSP_USING_BOARD_EH_SS6600XXX) || defined(BSP_USING_BOARD_EC_LB555_WATCH)
#include "pin_service.h"

static datac_handle_t pin_irq_service = DATA_CLIENT_INVALID_HANDLE;
static uint8_t pin_irq_service_connect_state = 0; // 0 - not connect, 1 - connecting, 2 - connected, 3 - connect error

static rt_err_t dk05e01t_f412_init(rt_device_t dev)
{
    rt_err_t ret = RT_EOK;
    diff = 0;
    LOG_I("dk05e01t_f412_init OK");
    return ret;
}

static rt_size_t wheel_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    if ((buffer != NULL) && (size >= 1  && size <= 4) && (diff != 0))
    {
        memcpy(buffer, &diff, size);
        LOG_I("read=%d", diff);

        diff = 0;
        return size;
    }

    return 0;
}

static int irq_pin_service_callback(data_callback_arg_t *arg)
{
    if (MSG_SERVICE_DATA_NTF_IND == arg->msg_id)
    {
        rt_err_t ret;
        pin_common_msg_t pin_msg;
        RT_ASSERT(arg != NULL);
        RT_ASSERT(sizeof(pin_common_msg_t) == arg->data_len);
        memcpy(&pin_msg, arg->data, arg->data_len);
        RT_ASSERT(pin_msg.id == ENCODE_INPUT_PIN_A);
        LOG_I("wheel irq_pin_service ntf ind");
        if (1 == rt_pin_read(ENCODE_INPUT_PIN_A))
        {
            int b = rt_pin_read(ENCODE_INPUT_PIN_B);
            if (0 == b)
                diff ++;
            else
                diff --;
            LOG_D("wheel pinB=%d", b);
        }
        else
        {
            //Invalid pinA state, ignore it.
            LOG_D("wheel invalid");
        }
        data_msg_t msg;
        pin_common_msg_t *p_pin_msg;
        p_pin_msg = (pin_common_msg_t *) data_service_init_msg(&msg, PIN_MSG_ENABLE_IRQ_REQ, sizeof(pin_common_msg_t));
        p_pin_msg->id = ENCODE_INPUT_PIN_A;

        ret = datac_send_msg(pin_irq_service, &msg);
        LOG_I("wheel irq on ret %d", ret);
    }
    else if (MSG_SERVICE_SUBSCRIBE_RSP == arg->msg_id)
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        LOG_I("wheel irq_pin_service ret %d", rsp->result);
        if (rsp->result == 0)
        {
            LOG_D("wheel irq_pin_service connected");
            pin_irq_service_connect_state = 2;
        }
        else
        {
            LOG_E("wheel irq_pin_service fail");
            pin_irq_service_connect_state = 3;
        }
    }

    return 0;
}

static rt_err_t wheel_open(struct rt_device *dev, rt_uint16_t oflag)
{
    rt_err_t ret;
    uint32_t retry_count = 0;

    pin_irq_service = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != pin_irq_service);
    do
    {
        if (1 != pin_irq_service_connect_state)
        {
            datac_subscribe(pin_irq_service,
#ifdef SOC_BF0_HCPU
                            "pin_l",
#else
                            "pin_h",
#endif /* SOC_BF0_HCPU */
                            irq_pin_service_callback, 0);
        }

        rt_thread_delay(RT_TICK_PER_SECOND / 100); //Let system breath

        if (3 == pin_irq_service_connect_state)
        {
            LOG_E("wheel irq_pin_service error, retry...%d", ++retry_count);
        }

        if (retry_count > 10)
        {
            LOG_E("Connect irq_pin_service timeout.");
            return RT_ERROR;
        }

    }
    while (2 != pin_irq_service_connect_state);


    pin_config_msg_t config;

    config.id = ENCODE_INPUT_PIN_A;
    config.mode = PIN_MODE_INPUT;
    config.irq_mode = PIN_IRQ_MODE_RISING;
    config.flag = PIN_SERVICE_FLAG_SET_IRQ_MODE | PIN_SERVICE_FLAG_AUTO_DISABLE_IRQ;

    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != pin_irq_service);
    ret = datac_config(pin_irq_service, sizeof(pin_config_msg_t), (uint8_t *)&config);
    LOG_I("wheel cfg ret %d", ret);

    data_msg_t msg;
    pin_common_msg_t *p_pin_msg;
    p_pin_msg = (pin_common_msg_t *) data_service_init_msg(&msg, PIN_MSG_ENABLE_IRQ_REQ, sizeof(pin_common_msg_t));
    p_pin_msg->id = ENCODE_INPUT_PIN_A;

    ret = datac_send_msg(pin_irq_service, &msg);

    LOG_I("wheel open ret %d", ret);
    return RT_EOK;

}

static rt_err_t wheel_close(struct rt_device *dev)
{
    rt_err_t ret;

    data_msg_t msg;
    pin_common_msg_t *p_pin_msg;
    p_pin_msg = (pin_common_msg_t *) data_service_init_msg(&msg, PIN_MSG_DISABLE_IRQ_REQ, sizeof(pin_common_msg_t));
    p_pin_msg->id = ENCODE_INPUT_PIN_A;
    datac_send_msg(pin_irq_service, &msg);

    p_pin_msg = (pin_common_msg_t *) data_service_init_msg(&msg, PIN_MSG_DETACH_IRQ_REQ, sizeof(pin_common_msg_t));

    p_pin_msg->id = ENCODE_INPUT_PIN_A;
    ret = datac_send_msg(pin_irq_service, &msg);


    if (DATA_CLIENT_INVALID_HANDLE != pin_irq_service)
    {
        /*
            Unsubscirbe RSP msg will fast than other request msg, and cause assert in data_service.c
            Delay 10ms for waitting previous service request rsp.
            Remove it if bug fixed.
        */

        rt_thread_mdelay(10);
        //datac_unsubscribe(pin_irq_service);
        datac_close(pin_irq_service);
        pin_irq_service_connect_state = 0;
        pin_irq_service = DATA_CLIENT_INVALID_HANDLE;
    }

    LOG_I("wheel close r=%d", ret);

    return RT_EOK;
}




#else


static void timeout_handler(void *param)
{
    if (1 == rt_pin_read(ENCODE_INPUT_PIN_A))
    {
        LOG_D("timeout_handler pinA=%d, pinB=%d", rt_pin_read(ENCODE_INPUT_PIN_A), rt_pin_read(ENCODE_INPUT_PIN_B));

        if (1 == rt_pin_read(ENCODE_INPUT_PIN_B))
            diff ++;
        else
            diff --;
    }
    else
    {
        //Invalid pinA state, ignore it.
    }

    rt_pin_irq_enable(ENCODE_INPUT_PIN_A, 1);
}

static void encoder_pin_irq_handler(void *arg)
{
    LOG_I("encoder_pin_irq_handler");

    rt_pin_irq_enable(ENCODE_INPUT_PIN_A, 0);
    rt_timer_stop(p_timer);
    rt_timer_start(p_timer);
}

static rt_size_t wheel_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    if ((buffer != NULL) && (size >= 1  && size <= 4) && (diff != 0))
    {
        memcpy(buffer, &diff, size);
        LOG_I("read=%d", diff);

        diff = 0;

        return size;
    }

    return 0;
}


static rt_err_t wheel_open(struct rt_device *dev, rt_uint16_t oflag)
{
    rt_pin_irq_enable(ENCODE_INPUT_PIN_A, 1);
    p_timer = rt_timer_create("wheel", timeout_handler, 0, rt_tick_from_millisecond(1), RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_HARD_TIMER);
    RT_ASSERT(p_timer != NULL);


    LOG_I("wheel open");
    return RT_EOK;

}

static rt_err_t wheel_close(struct rt_device *dev)
{
    rt_pin_irq_enable(ENCODE_INPUT_PIN_A, 0);


    LOG_I("wheel close");

    return RT_EOK;
}


static rt_err_t dk05e01t_f412_init(rt_device_t dev)
{
    rt_err_t ret = RT_EOK;


    rt_pin_mode(ENCODE_INPUT_PIN_A, PIN_MODE_INPUT);
    rt_pin_mode(ENCODE_INPUT_PIN_B, PIN_MODE_INPUT);
    rt_pin_attach_irq(ENCODE_INPUT_PIN_A, PIN_IRQ_MODE_RISING, encoder_pin_irq_handler, NULL);


    diff = 0;


    LOG_I("dk05e01t_f412_init OK");

    return ret;
}

#endif

static int regist_device(void)
{

    memset(&this_device, 0, sizeof(this_device));

    this_device.type = RT_Device_Class_Char;
    this_device.init = dk05e01t_f412_init;
    this_device.read = wheel_read;
    this_device.open = wheel_open;
    this_device.close = wheel_close;


    /* register graphic device driver */
    rt_device_register(&this_device, "wheel",
                       RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_STANDALONE);

    return 0;
}

INIT_DEVICE_EXPORT(regist_device);

