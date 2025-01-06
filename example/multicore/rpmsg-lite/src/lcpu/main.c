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


#include "rpmsg_lite.h"
#include "ipc_config.h"


static rt_mq_t test_rpmsg_mq;
static struct rpmsg_lite_endpoint *my_ept;
static struct rpmsg_lite_instance *my_rpmsg;

static int32_t rpmsg_rx_cb(void *payload, uint32_t payload_len, uint32_t src, void *priv);


int main(void)
{
    rt_err_t result;
    rpmsg_rx_cb_data_t rx_data;
    int32_t r;
    int32_t timeout;
    rt_base_t tick;

    test_rpmsg_mq = rt_mq_create("test_rpmsg_mq", sizeof(rpmsg_rx_cb_data_t), 30, RT_IPC_FLAG_FIFO);
    RT_ASSERT(test_rpmsg_mq);

    my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_BUF_ADDR_REMOTE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
    RT_ASSERT(my_rpmsg);

    r = rpmsg_lite_wait_for_link_up(my_rpmsg, 1000);
    RT_ASSERT(RL_TRUE == (uint32_t)r);

    my_ept = rpmsg_lite_create_ept(my_rpmsg, REMOTE_EPT_ADDR, rpmsg_rx_cb, (void *)1);
    RT_ASSERT(my_ept);

    timeout = 6000;
    while (1)
    {
        tick = rt_tick_get();
        result = rt_mq_recv(test_rpmsg_mq, &rx_data, sizeof(rx_data), timeout);
        if ((RT_EOK == result) && (rx_data.len > 0))
        {
            uint8_t *buf;
            buf = rt_malloc(rx_data.len);
            RT_ASSERT(buf);
            memcpy((void *)buf, (void *)rx_data.data, rx_data.len);
            r = rpmsg_lite_release_rx_buffer(my_rpmsg, rx_data.data);
            RT_ASSERT(RL_SUCCESS == r);
            rt_kprintf("from: %d\n", rx_data.src);
            rt_kprintf("rx: %s\n", buf);
            rt_free(buf);
        }
        tick = rt_tick_get() - tick;
        if (tick < timeout)
        {
            timeout -= tick;
        }
        else
        {
            timeout = 6000;
            r = rpmsg_lite_send(my_rpmsg, my_ept, MASTER_EPT_ADDR, "hello_from_lcpu", strlen("hello_from_lcpu") + 1, 1000);
            RT_ASSERT(RL_SUCCESS == r);
        }
    }

    return RT_EOK;
}

static int32_t rpmsg_rx_cb(void *payload, uint32_t payload_len, uint32_t src, void *priv)
{
    rpmsg_rx_cb_data_t msg;

    RL_ASSERT(priv != RL_NULL);

    msg.data = payload;
    msg.len  = payload_len;
    msg.src  = src;
    msg.priv = priv;

    /* if message is successfully added into queue then hold rpmsg buffer */
    if (RT_EOK == rt_mq_send(test_rpmsg_mq, &msg, sizeof(msg)))
    {
        /* hold the rx buffer */
        return RL_HOLD;
    }

    return RL_RELEASE;
}


static int send(int argc, char *argv[])
{
    int32_t r;
    size_t msg_size;

    if (argc < 2)
    {
        rt_kprintf("wrong argument\n");
        return -1;
    }
    /* including null terminator */
    msg_size = strlen(argv[1]) + 1;
    r = rpmsg_lite_send(my_rpmsg, my_ept, MASTER_EPT_ADDR, argv[1], msg_size, 1000);
    RT_ASSERT(RL_SUCCESS == r);

    return 0;
}
MSH_CMD_EXPORT(send, "send msg to hcpu")

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

