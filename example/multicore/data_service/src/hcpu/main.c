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

#include "data_service_subscriber.h"
#include "test_service.h"

static datac_handle_t client_handle;

static int test_service_callback(data_callback_arg_t *arg)
{
    if (MSG_SERVICE_DATA_NTF_IND == arg->msg_id)
    {
        test_service_data_ntf_ind_t *data_ntf_ind;

        RT_ASSERT(arg->data_len == sizeof(test_service_data_ntf_ind_t));
        data_ntf_ind = (test_service_data_ntf_ind_t *)arg->data;

        RT_ASSERT(data_ntf_ind);

        rt_kprintf("data ntf:%d\n", data_ntf_ind->data);
    }
    else if (MSG_SERVICE_TEST_DATA_RSP == arg->msg_id)
    {
        test_service_data_rsp_t *data_rsp;

        RT_ASSERT(arg->data_len == sizeof(test_service_data_rsp_t));
        data_rsp = (test_service_data_rsp_t *)arg->data;

        RT_ASSERT(data_rsp);

        rt_kprintf("data rsp:%d\n", data_rsp->data);
    }
    else if (MSG_SERVICE_TEST_DATA2_IND == arg->msg_id)
    {
        test_service_data2_ind_t *data2_ind;

        RT_ASSERT(arg->data_len == sizeof(test_service_data2_ind_t));
        data2_ind = (test_service_data2_ind_t *)arg->data;

        RT_ASSERT(data2_ind);

        rt_kprintf("data2 ind:%d\n", data2_ind->data);
    }
    else if (MSG_SERVICE_SUBSCRIBE_RSP == arg->msg_id)
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        RT_ASSERT(client_handle == rsp->handle);
        rt_kprintf("Subscribe result:%d\n", rsp->result);
    }

    return 0;
}

static int request(int argc, char *argv[])
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err;

    body = data_service_init_msg(&msg, MSG_SERVICE_TEST_DATA_REQ, 0);
    err = datac_send_msg(client_handle, &msg);
    RT_ASSERT(RT_EOK == err);

    return 0;
}
MSH_CMD_EXPORT(request, "request data")

int main(void)
{
    client_handle = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != client_handle);
    datac_subscribe(client_handle, "test", test_service_callback, 0);

    while (1)
    {
        rt_thread_mdelay(100000);
    }

    return RT_EOK;
}



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

