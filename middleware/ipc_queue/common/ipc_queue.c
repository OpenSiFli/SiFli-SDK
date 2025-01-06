/**
  ******************************************************************************
  * @file   ipc_queue.c
  * @author Sifli software development team
  * @brief Sibles ipc library source.
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


#include <stdint.h>
#include <string.h>
#include "ipc_queue.h"
#include "ipc_hw.h"
#include "circular_buf.h"
//#include "log.h"

#ifndef __ROM_USED
    #define __ROM_USED
#endif

typedef struct
{
    bool active;                          /**< whether the queue is opened, true: opened  */
    uint32_t data_len;                    /**< len of data in rx_ring_buffer */
    struct circular_buf *rx_ring_buffer;
    struct circular_buf *tx_ring_buffer;
    ipc_hw_q_handle_t hw_q_handle;        /**< handle of hw queue */
    ipc_queue_cfg_t cfg;                  /**< queue configuration */
} ipc_queue_t;

#define IPC_LOGICAL_QUEUE_NUM    (4)
#define IPC_INVALID_QUEUE_ID     (UINT8_MAX)
#define IPC_QUEUE_HANDLE_OFFSET  (10)
#define IPC_QUEUE_HANDLE_2_OFFSET(handle)  ((handle) - IPC_QUEUE_HANDLE_OFFSET)
#define IPC_QUEUE_OFFSET_2_HANDLE(offset)  ((offset) + IPC_QUEUE_HANDLE_OFFSET)

typedef struct
{
    bool init;
    ipc_queue_t queues[IPC_LOGICAL_QUEUE_NUM];
} ipc_ctx_t;

__ROM_USED ipc_ctx_t ipc_ctx;


static bool is_valid_handle(ipc_queue_handle_t handle)
{
    bool valid = false;
    int32_t offset;

    if (!ipc_ctx.init)
    {
        return valid;
    }

    if ((handle < IPC_QUEUE_HANDLE_OFFSET)
            || ((handle - IPC_QUEUE_HANDLE_OFFSET) >= IPC_LOGICAL_QUEUE_NUM))
    {
        return valid;
    }

    offset = IPC_QUEUE_HANDLE_2_OFFSET(handle);
    if (IPC_INVALID_QUEUE_ID == ipc_ctx.queues[offset].cfg.qid)
    {
        return valid;
    }

    valid = true;

    return valid;
}

__ROM_USED ipc_queue_handle_t ipc_queue_init(ipc_queue_cfg_t *q_cfg)
{
    uint32_t i;
    ipc_queue_handle_t handle = IPC_QUEUE_INVALID_HANDLE;
    int32_t offset;
    ipc_queue_t *ipc_queue;
    uint32_t mask;

    if (NULL == q_cfg)
    {
        return handle;
    }

    mask = os_interrupt_disable();

    /* initialize ipc queues */
    if (!ipc_ctx.init)
    {
        for (i = 0; i < IPC_LOGICAL_QUEUE_NUM; i++)
        {
            ipc_ctx.queues[i].cfg.qid = IPC_INVALID_QUEUE_ID;
            ipc_ctx.queues[i].active = false;
        }
        ipc_ctx.init = true;
    }

    ipc_queue = &ipc_ctx.queues[0];
    offset = -1;
    for (i = 0; i < IPC_LOGICAL_QUEUE_NUM; i++)
    {
        if (q_cfg->qid == ipc_queue->cfg.qid)
        {
            /* qid already present */
            handle = IPC_QUEUE_INVALID_HANDLE;
            break;
        }
        if ((IPC_QUEUE_INVALID_HANDLE == handle)
                && (IPC_INVALID_QUEUE_ID == ipc_queue->cfg.qid))
        {
            /* first vacant found */
            offset = i;
            handle = IPC_QUEUE_OFFSET_2_HANDLE(offset);
        }
        ipc_queue++;
    }

    if ((i >= IPC_LOGICAL_QUEUE_NUM) && (offset >= 0))
    {
        /* new qid and vacant found */
        memcpy(&ipc_ctx.queues[offset].cfg, q_cfg, sizeof(*q_cfg));
        ipc_ctx.queues[offset].data_len = 0;
        ipc_ctx.queues[offset].active = false;
    }

    os_interrupt_enable(mask);

    return handle;
}

__ROM_USED int32_t ipc_queue_get_user_data(ipc_queue_handle_t handle, uint32_t *user_data)
{
    int32_t result = -1;
    ipc_queue_t *queue;
    uint32_t mask;
    int32_t offset;

    if (!user_data)
    {
        return result;
    }

    if (!is_valid_handle(handle))
    {
        return result;
    }
    offset = IPC_QUEUE_HANDLE_2_OFFSET(handle);
    queue = &ipc_ctx.queues[offset];

    mask = os_interrupt_disable();

    *user_data = queue->cfg.user_data;

    result = 0;
    os_interrupt_enable(mask);

    return result;
}

__ROM_USED int32_t ipc_queue_set_user_data(ipc_queue_handle_t handle, uint32_t user_data)
{
    int32_t result = -1;
    ipc_queue_t *queue;
    uint32_t mask;
    int32_t offset;

    if (!user_data)
    {
        return result;
    }

    if (!is_valid_handle(handle))
    {
        return result;
    }
    offset = IPC_QUEUE_HANDLE_2_OFFSET(handle);
    queue = &ipc_ctx.queues[offset];

    mask = os_interrupt_disable();

    queue->cfg.user_data = user_data;

    result = 0;
    os_interrupt_enable(mask);

    return result;
}

__ROM_USED int32_t ipc_queue_open(ipc_queue_handle_t handle)
{
    int32_t result = -1;
    uint8_t *pool;
    uint32_t mask;
    ipc_queue_t *queue;
    int32_t offset;

    if (!is_valid_handle(handle))
    {
        return result;
    }
    offset = IPC_QUEUE_HANDLE_2_OFFSET(handle);
    queue = &ipc_ctx.queues[offset];

    mask = os_interrupt_disable();

    if (queue->active)
    {
        goto __END;
    }

    /* receiver no need to initialize the buffer as sender has already init the buffer */
    queue->rx_ring_buffer = (struct circular_buf *)queue->cfg.rx_buf_addr;

    queue->tx_ring_buffer = (struct circular_buf *)queue->cfg.tx_buf_addr;

    if (queue->tx_ring_buffer)
    {
        SF_ASSERT(queue->cfg.tx_buf_size > sizeof(struct circular_buf));
        /* sender initialize the buffer */
        pool = (uint8_t *)(queue->tx_ring_buffer + 1);
        circular_buf_wr_init(queue->tx_ring_buffer, pool, queue->cfg.tx_buf_size - sizeof(struct circular_buf));

        /* update rd_buf_ptr */
        pool = (uint8_t *)((struct circular_buf *)queue->cfg.tx_buf_addr_alias + 1);
        circular_buf_rd_init(queue->tx_ring_buffer, pool, queue->cfg.tx_buf_size - sizeof(struct circular_buf));
    }
    result = ipc_hw_enable_interrupt(&queue->hw_q_handle, queue->cfg.qid, (uint32_t)handle);

    if (0 == result)
    {
#if 0
#ifdef RT_USING_PM
        rt_pm_device_register((struct rt_device *)queue, &ipc_queue_pm_op);
#endif /* RT_USING_PM */
#endif
        queue->active = true;
    }

__END:
    os_interrupt_enable(mask);

    return result;
}

__ROM_USED int32_t ipc_queue_open2(ipc_queue_handle_t handle)
{
    int32_t result = -1;
    uint8_t *pool;
    uint32_t mask;
    ipc_queue_t *queue;
    int32_t offset;

    if (!is_valid_handle(handle))
    {
        return result;
    }
    offset = IPC_QUEUE_HANDLE_2_OFFSET(handle);
    queue = &ipc_ctx.queues[offset];

    mask = os_interrupt_disable();

    if (queue->active)
    {
        goto __END;
    }

    /* receiver no need to initialize the buffer as sender has already init the buffer */
    queue->rx_ring_buffer = (struct circular_buf *)queue->cfg.rx_buf_addr;

    queue->tx_ring_buffer = (struct circular_buf *)queue->cfg.tx_buf_addr;

    if (queue->tx_ring_buffer)
    {
        SF_ASSERT(queue->cfg.tx_buf_size > sizeof(struct circular_buf));
        /* sender initialize the buffer */
        pool = (uint8_t *)(queue->tx_ring_buffer + 1);
        circular_buf_wr_init(queue->tx_ring_buffer, pool, queue->cfg.tx_buf_size - sizeof(struct circular_buf));

        /* update rd_buf_ptr */
        pool = (uint8_t *)((struct circular_buf *)queue->cfg.tx_buf_addr_alias + 1);
        circular_buf_rd_init(queue->tx_ring_buffer, pool, queue->cfg.tx_buf_size - sizeof(struct circular_buf));
    }
    result = ipc_hw_enable_interrupt2(&queue->hw_q_handle, queue->cfg.qid, (uint32_t)handle);

    if (0 == result)
    {
#if 0
#ifdef RT_USING_PM
        rt_pm_device_register((struct rt_device *)queue, &ipc_queue_pm_op);
#endif /* RT_USING_PM */
#endif
        queue->active = true;
    }

__END:
    os_interrupt_enable(mask);

    return result;
}

__ROM_USED bool ipc_queue_is_open(ipc_queue_handle_t handle)
{
    ipc_queue_t *queue;
    int32_t offset;

    if (!is_valid_handle(handle))
    {
        return false;
    }
    offset = IPC_QUEUE_HANDLE_2_OFFSET(handle);
    queue = &ipc_ctx.queues[offset];

    return queue->active;
}

__ROM_USED int32_t ipc_queue_close(ipc_queue_handle_t handle)
{
    int32_t result = -1;
    ipc_queue_t *queue;
    uint32_t mask;
    int32_t offset;

    if (!is_valid_handle(handle))
    {
        return result;
    }
    offset = IPC_QUEUE_HANDLE_2_OFFSET(handle);
    queue = &ipc_ctx.queues[offset];

    if (!queue->active)
    {
        return result;
    }

    mask = os_interrupt_disable();
    result = ipc_hw_disable_interrupt(&queue->hw_q_handle);
    queue->active = false;
    queue->rx_ring_buffer = NULL;
    queue->tx_ring_buffer = NULL;

    os_interrupt_enable(mask);

    return result;
}

__ROM_USED int32_t ipc_queue_close2(ipc_queue_handle_t handle)
{
    int32_t result = -1;
    ipc_queue_t *queue;
    uint32_t mask;
    int32_t offset;

    if (!is_valid_handle(handle))
    {
        return result;
    }
    offset = IPC_QUEUE_HANDLE_2_OFFSET(handle);
    queue = &ipc_ctx.queues[offset];

    if (!queue->active)
    {
        return result;
    }

    mask = os_interrupt_disable();
    result = ipc_hw_disable_interrupt2(&queue->hw_q_handle);
    queue->active = false;
    queue->rx_ring_buffer = NULL;
    queue->tx_ring_buffer = NULL;

    os_interrupt_enable(mask);

    return result;
}

__ROM_USED int32_t ipc_queue_deinit(ipc_queue_handle_t handle)
{
    int32_t result = -1;
    ipc_queue_t *queue;
    int32_t offset;

    if (!is_valid_handle(handle))
    {
        return result;
    }

    offset = IPC_QUEUE_HANDLE_2_OFFSET(handle);
    queue = &ipc_ctx.queues[offset];

    if (queue->active)
    {
        return result;
    }
    queue->cfg.qid = IPC_INVALID_QUEUE_ID;

    result = 0;

    return result;
}

__ROM_USED size_t ipc_queue_read(ipc_queue_handle_t handle, void *buffer, size_t size)
{
    ipc_queue_t *queue;
    size_t data_len;
    int32_t offset;

    if (!is_valid_handle(handle))
    {
        return 0;
    }

    if (NULL == buffer)
    {
        return 0;
    }

    offset = IPC_QUEUE_HANDLE_2_OFFSET(handle);
    queue = &ipc_ctx.queues[offset];

    if (!queue->active)
    {
        return 0;
    }

    if (!queue->rx_ring_buffer)
    {
        return 0;
    }

    if (0 == queue->data_len)
    {
        return 0;
    }

    data_len = circular_buf_get_and_update_len(queue->rx_ring_buffer, buffer, size, (size_t *)&queue->data_len);

    return data_len;
}

__ROM_USED size_t ipc_queue_write(ipc_queue_handle_t handle, const void *buffer, size_t size, uint32_t timeout)
{
    size_t data_len;
    uint32_t start_time;
    uint32_t cnt;
    uint32_t total_len;
    ipc_queue_t *queue;
    int32_t offset;

    if (!is_valid_handle(handle))
    {
        return 0;
    }

    offset = IPC_QUEUE_HANDLE_2_OFFSET(handle);
    queue = &ipc_ctx.queues[offset];
    if (!queue->active)
    {
        return 0;
    }

    if (NULL == buffer)
    {
        /* just trigger interrupt */
        ipc_hw_trigger_interrupt(&queue->hw_q_handle);
        return 0;
    }

    if (!queue->tx_ring_buffer)
    {
        return 0;
    }

    cnt = 0;
    start_time = HAL_GetTick();
    total_len = size;
    while (total_len > 0)
    {
        data_len = circular_buf_put(queue->tx_ring_buffer, buffer, total_len);
        SF_ASSERT(data_len <= total_len);
        total_len -= data_len;
        buffer = (uint8_t *)buffer + data_len;

        if (data_len > 0)
        {
            /* trigger mailbox if new data has been written */
            ipc_hw_trigger_interrupt(&queue->hw_q_handle);
        }
        if (HAL_GetTick() != start_time)
        {
            cnt++;
            start_time = HAL_GetTick();
        }
        if (cnt >= timeout)
        {
            break;
        }
    }

    return (size - total_len);
}

__ROM_USED bool ipc_queue_check_idle(void)
{
    bool is_idle = true;
    uint32_t i;
    ipc_queue_t *q;
    uint32_t mask;

    if (!ipc_ctx.init)
    {
        goto __END;
    }

    mask = os_interrupt_disable();
    q = &ipc_ctx.queues[0];
    for (i = 0; i < IPC_LOGICAL_QUEUE_NUM; i++)
    {
        /* interrupt has not been handled by receiver */
        if (q->active && (0 != ipc_hw_check_interrupt(&q->hw_q_handle)))
        {
            is_idle = false;
            break;
        }
        if (q->active && q->tx_ring_buffer
                && circular_buf_data_len(q->tx_ring_buffer) > 0)
        {
            is_idle = false;
            break;
        }
        q++;
    }
    os_interrupt_enable(mask);

__END:

    return is_idle;
}

__WEAK bool ipc_queue_check_idle_rom(void)
{
    return ipc_queue_check_idle();
}

__ROM_USED void ipc_queue_restore_all(void)
{
    ipc_ctx_t *ctx = &ipc_ctx;
    uint32_t i;
    int32_t res;

    if (!ctx->init)
    {
        return;
    }


    for (i = 0; i < IPC_LOGICAL_QUEUE_NUM; i++)
    {
        if (ctx->queues[i].active)
        {
            ctx->queues[i].active = false;
            res = ipc_queue_open(IPC_QUEUE_OFFSET_2_HANDLE(i));
            SF_ASSERT(0 == res);
        }
    }

    return ;
}

__WEAK void ipc_queue_restore_all_rom(void)
{
    ipc_queue_restore_all();
}

__ROM_USED void ipc_queue_data_ind(uint32_t user_data)
{
    ipc_queue_handle_t handle = (ipc_queue_handle_t)user_data;
    ipc_queue_t *queue;
    int32_t offset;

    if (!is_valid_handle(handle))
    {
        return;
    }
    offset = IPC_QUEUE_HANDLE_2_OFFSET(handle);
    queue = &ipc_ctx.queues[offset];
    if (!queue->active)
    {
        return;
    }

    if (queue->rx_ring_buffer)
    {
        queue->data_len = circular_buf_data_len(queue->rx_ring_buffer);
    }
    else
    {
        queue->data_len = 0;
    }
    queue->cfg.rx_ind(handle, queue->data_len);
}


__ROM_USED size_t ipc_queue_get_rx_size(ipc_queue_handle_t handle)
{
    ipc_queue_t *queue;
    int32_t offset;

    if (!is_valid_handle(handle))
    {
        return 0;
    }

    offset = IPC_QUEUE_HANDLE_2_OFFSET(handle);
    queue = &ipc_ctx.queues[offset];

    if (!queue->active)
    {
        return 0;
    }

    if (!queue->rx_ring_buffer)
    {
        return 0;
    }

    return queue->data_len;
}



