/**
  ******************************************************************************
  * @file   main.c
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

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>

#define LOG_TAG "main"
#include "log.h"
#include "data_service_provider.h"
#include "test_service.h"

static datas_handle_t test_service_handle;
static uint32_t test_counter;

static int32_t test_service_msg_handler(datas_handle_t service, data_msg_t *msg);


static data_service_config_t test_service_cb =
{
    .max_client_num = 1,
    .queue = RT_NULL, /* share the same queue of data service process thread */
    .data_filter = NULL,
    .msg_handler = test_service_msg_handler,
};


static int32_t test_service_msg_handler(datas_handle_t service, data_msg_t *msg)
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
    case MSG_SERVICE_TEST_DATA_REQ:
    {
        rt_err_t r;
        test_service_data_rsp_t data_rsp;

        rt_kprintf("receive msg DATA_REQ\n");
        test_counter++;
        data_rsp.data = test_counter;
        r = datas_send_response_data(service, msg, sizeof(data_rsp), (uint8_t *)&data_rsp);
        RT_ASSERT(RT_EOK == r);
        break;
    }
    case MSG_SERVICE_DATA_RDY_IND:
    {
        data_rdy_ind_t *data_rdy_ind = (data_rdy_ind_t *)(data_service_get_msg_body(msg));
        int32_t result;
        test_service_data_ntf_ind_t data_ntf_ind;

        RT_ASSERT(data_rdy_ind);

        rt_kprintf("push DATA_NTF_IND to all clients\n");
        data_ntf_ind.data = (uint32_t)data_rdy_ind->data;
        result = datas_push_data_to_client(service, sizeof(data_ntf_ind), (uint8_t *)&data_ntf_ind);
        RT_ASSERT(0 == result);
        break;
    }
    default:
    {
        RT_ASSERT(0);
    }
    }

    return 0;
}

static int test_service_register(void)
{
    /* register test service*/
    test_service_handle = datas_register("test", &test_service_cb);
    RT_ASSERT(test_service_handle);

    return 0;
}
INIT_COMPONENT_EXPORT(test_service_register);


static int trigger(int argc, char *argv[])
{
    rt_err_t err;

    test_counter++;
    err = datas_data_ready(test_service_handle, sizeof(test_counter), (uint8_t *)test_counter);
    RT_ASSERT(RT_EOK == err);

    return 0;
}
MSH_CMD_EXPORT(trigger, "trigger notification to client")

static int trigger2(int argc, char *argv[])
{
    int32_t result;
    test_service_data2_ind_t data2_ind;

    test_counter++;

    data2_ind.data = test_counter;
    result = datas_push_msg_to_client(test_service_handle, MSG_SERVICE_TEST_DATA2_IND, sizeof(data2_ind), (uint8_t *)&data2_ind);
    RT_ASSERT(0 == result);

    return 0;
}
MSH_CMD_EXPORT(trigger2, "trigger data2_ind to client")



int main(void)
{
    uint32_t cnt = 0;
    while (1)
    {
        rt_thread_mdelay(5000);
        if (cnt & 1)
        {
            trigger(0, NULL);
        }
        else
        {
            trigger2(0, NULL);
        }
        cnt++;
    }

    return RT_EOK;
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

