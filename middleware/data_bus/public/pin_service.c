/**
  ******************************************************************************
  * @file   pin_service.c
  * @author Sifli software development team
  * @brief  Use pins of another core.
  *
* *****************************************************************************
**/
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

#include "data_service_provider.h"
#ifdef RT_USING_PIN
#include "pin_service.h"
#ifndef _MSC_VER
    #include "drv_gpio.h"
#endif
#include "string.h"

#define LOG_TAG  "svc.pin"
#include "log.h"

#ifdef _MSC_VER
    #define PIN_BELONG_THIS_CORE(pin) (0)
#else
    #ifdef SOC_BF0_HCPU
        #define PIN_BELONG_THIS_CORE(pin) (GET_GPIO_INSTANCE(pin) == hwp_gpio1)
    #else
        #define PIN_BELONG_THIS_CORE(pin) (GET_GPIO_INSTANCE(pin) == hwp_gpio2)
    #endif /* SOC_BF0_HCPU */
#endif /*_MSC_VER*/

#define TO_ARGUMENT(pin_id,flag)  ((uint32_t) (((flag)&0xFFFF)<<16 | ((pin_id)&0xFFFF)))
#define GET_ID_FROM_ARG(arg)   ((uint16_t) (((rt_uint32_t)arg) & 0xFFFF ))
#define GET_FLAG_FROM_ARG(arg) ((uint16_t) ((((rt_uint32_t)arg)>>16) & 0xFFFF ))

static datas_handle_t this_service = NULL;
static bool service_filter(data_req_t *config, uint16_t msg_id, uint32_t len, uint8_t *data)
{
    bool ret = false;
    switch (msg_id)
    {
    case MSG_SERVICE_DATA_NTF_IND:
    {
        pin_config_msg_t *p_config = (pin_config_msg_t *)&config->data[0];
        RT_ASSERT(p_config != NULL);
        RT_ASSERT(PIN_BELONG_THIS_CORE(p_config->id));

        pin_common_msg_t *p_push_msg = (pin_common_msg_t *)data;

        ret = (p_push_msg->id == p_config->id);
        break;
    }

    default:
        RT_ASSERT(0);
        break;
    }

    return ret;
}

static void pin_irq_handler(void *arg)
{
    uint16_t pin  =  GET_ID_FROM_ARG(arg);
    uint16_t flag =  GET_FLAG_FROM_ARG(arg);

    RT_ASSERT(PIN_BELONG_THIS_CORE(pin));

    LOG_D("pin_irq_handler pin=%d, f=%x\n", pin, flag);

    if (flag & PIN_SERVICE_FLAG_AUTO_DISABLE_IRQ)
        rt_pin_irq_enable(pin, 0);

    datas_data_ready(this_service, sizeof(uint8_t *), (uint8_t *)arg);
}




static int32_t msg_handler(datas_handle_t service, data_msg_t *msg)
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
        rt_err_t result;
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        pin_config_msg_t *p_config = (pin_config_msg_t *)&req->data[0];
        RT_ASSERT(p_config != NULL);
        RT_ASSERT(PIN_BELONG_THIS_CORE(p_config->id));

        rt_pin_mode(p_config->id, p_config->mode);

        if (p_config->flag & PIN_SERVICE_FLAG_SET_IRQ_MODE)
        {
            uint32_t arg = TO_ARGUMENT(p_config->id, p_config->flag);
            result = rt_pin_attach_irq(p_config->id, p_config->irq_mode, pin_irq_handler, (void *)arg);
        }

        datas_send_response(service, msg, result);
        break;
    }

    case MSG_SERVICE_DATA_RDY_IND: //Pin irq happened
    {
        data_rdy_ind_t *data_ind = (data_rdy_ind_t *)(data_service_get_msg_body(msg));
        uint16_t pin;

        RT_ASSERT(data_ind);
        pin  =  GET_ID_FROM_ARG(data_ind->data);
        RT_ASSERT(PIN_BELONG_THIS_CORE(pin));

        pin_common_msg_t push_msg;

        push_msg.id = pin;
        push_msg.value = 0;

        datas_push_data_to_client(service, sizeof(push_msg), (uint8_t *)&push_msg);
        break;
    }

    case PIN_MSG_ENABLE_IRQ_REQ:
    case PIN_MSG_DISABLE_IRQ_REQ:
    case PIN_MSG_DETACH_IRQ_REQ:
    case PIN_MSG_VALUE_GET_REQ:
    case PIN_MSG_VALUE_SET_REQ:
    {
        pin_common_msg_t *p_pin_msg = (pin_common_msg_t *)data_service_get_msg_body(msg);
        pin_common_msg_t pin_msg;
        rt_err_t result;

        RT_ASSERT(p_pin_msg);
        memcpy(&pin_msg, p_pin_msg, sizeof(pin_msg));
        RT_ASSERT(PIN_BELONG_THIS_CORE(pin_msg.id));


        switch (msg->msg_id)
        {
        case PIN_MSG_ENABLE_IRQ_REQ:
        {
            result = rt_pin_irq_enable(pin_msg.id, 1);
            datas_send_response(service, msg, result);
            break;
        }

        case PIN_MSG_DISABLE_IRQ_REQ:
        {
            result = rt_pin_irq_enable(pin_msg.id, 0);
            datas_send_response(service, msg, result);
            break;
        }

        case PIN_MSG_DETACH_IRQ_REQ:
        {
            result = rt_pin_detach_irq(pin_msg.id);
            datas_send_response(service, msg, result);
            break;
        }

        case PIN_MSG_VALUE_SET_REQ:
        {
            rt_pin_write(pin_msg.id, pin_msg.value);
            result = RT_EOK;
            datas_send_response(service, msg, result);
            break;
        }

        case PIN_MSG_VALUE_GET_REQ:
        {
            pin_msg.value = rt_pin_read(pin_msg.id);
            datas_send_response_data(service, msg, sizeof(pin_msg), (uint8_t *)&pin_msg);
            break;
        }

        default:
            RT_ASSERT(0);
        }

        break;
    }


    default:
    {
        RT_ASSERT(0);
    }
    }

    return 0;
}


static data_service_config_t pin_service_cb =
{
    .max_client_num = 50,
    .queue = RT_NULL,
    .data_filter = service_filter,
    .msg_handler = msg_handler,
};

#ifndef _MSC_VER
    static
#endif
int pin_service_register(void)
{
    rt_err_t ret = RT_EOK;

    this_service = datas_register(
#ifdef SOC_BF0_HCPU
                       "pin_h",
#else
                       "pin_l",
#endif /* SOC_BF0_HCPU */
                       &pin_service_cb);


    return ret;
}

INIT_COMPONENT_EXPORT(pin_service_register);
#endif /* RT_USING_PIN */
