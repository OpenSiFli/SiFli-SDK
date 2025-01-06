/**
  ******************************************************************************
  * @file   os_adaptor_rtthread.h
  * @author Sifli software development team
  * @brief Header file - OS adatpor interface.
 *
  ******************************************************************************
*/
/*
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

#include "os_adaptor.h"
#include "string.h"

__ROM_USED os_thread_t os_thread_create_int(const char *name, os_thread_func_t entry, void *parameter, void *stack_memory, uint32_t stack_size, os_priority_t priority, uint32_t tick)
{
    os_handle_t tid = RT_NULL;
    do
    {
        tid = rt_malloc(sizeof(os_handle));
        if (tid == RT_NULL)
            break;
        if (stack_memory != NULL)
        {
            tid->handle = rt_malloc(sizeof(struct rt_thread));
            RT_ASSERT(tid->handle);
            memset(tid->handle, 0, sizeof(struct rt_thread));
            tid->flag = RT_OS_ADAPTOR_STATIC;
            if (tid->handle != RT_NULL)
            {
                rt_thread_init((struct rt_thread *)(tid->handle), name, entry, parameter, stack_memory, stack_size, priority, tick);
            }
        }
        else
        {
            tid->handle = (void *)rt_thread_create(name, entry, parameter, stack_size, priority, tick);
            tid->flag = RT_OS_ADAPTOR_DYNAMIC;
        }
        if (tid->handle != RT_NULL)
            rt_thread_startup((struct rt_thread *)(tid->handle));
        else
        {
            rt_free(tid);
            tid = NULL;
        }
    }
    while (0);

    return (os_thread_t)tid;

}

__ROM_USED rt_err_t os_thread_delete(os_thread_t thread)
{
    os_handle_t tid = (os_handle_t)thread;

    if (tid == RT_NULL) return RT_EINVAL;


    if (RT_OS_ADAPTOR_STATIC == tid->flag)
    {
        if (tid->handle)
        {
            rt_thread_t t = (struct rt_thread *)(tid->handle);
            //Make sure thread was detached
            RT_ASSERT(NULL == rt_thread_find(&t->name[0]));

            rt_free(tid->handle);
            tid->handle = NULL;
        }
    }
    else
    {
        rt_thread_delete(tid->handle);
    }

    rt_free(tid);

    return RT_EOK;
}

__ROM_USED rt_err_t rt_timer_start_int(rt_timer_t timer_id, uint32_t duration)
{
    rt_tick_t ticks;

    ticks = rt_tick_from_millisecond(duration);
    rt_timer_control(timer_id, RT_TIMER_CTRL_SET_TIME, &ticks);
    return rt_timer_start(timer_id);
}

#ifdef RT_USING_MESSAGEQUEUE
__ROM_USED os_message_queue_t os_message_queue_create_int(const char *name, uint32_t max_count, uint32_t msg_size, void *mem_pool, uint32_t pool_size)
{
    os_handle_t queue = RT_NULL;
    do
    {
        queue = rt_malloc(sizeof(os_handle));
        if (queue == RT_NULL)
            break;
        if (mem_pool != RT_NULL)
        {
            queue->handle = rt_malloc(sizeof(struct rt_messagequeue));
            queue->flag = RT_OS_ADAPTOR_STATIC;
            if (queue->handle != RT_NULL)
            {
                rt_mq_init((struct rt_messagequeue *)(queue->handle), name, mem_pool, msg_size, pool_size, RT_IPC_FLAG_FIFO);
            }
        }
        else
        {
            queue->handle = (void *)rt_mq_create(name, msg_size, max_count, RT_IPC_FLAG_FIFO);
            queue->flag = RT_OS_ADAPTOR_DYNAMIC;
        }

        if (queue->handle == RT_NULL)
        {
            rt_free(queue);
            queue = NULL;
        }

    }
    while (0);
    return (os_message_queue_t)queue;
}


__ROM_USED rt_err_t os_message_delele_int(os_message_queue_t queue)
{
    rt_err_t err = RT_ERROR;
    os_handle_t q_cb = (os_handle_t)queue;
    if (q_cb->flag == RT_OS_ADAPTOR_STATIC)
        err = rt_mq_detach((rt_mq_t)(q_cb->handle));
    else if (q_cb->flag == RT_OS_ADAPTOR_DYNAMIC)
        err = rt_mq_delete((rt_mq_t)(q_cb->handle));
    rt_free(queue);
    return err;
}
#endif

#ifdef RT_USING_MAILBOX
__ROM_USED os_mailbox_t os_mailbox_create_int(const char *name, uint32_t size, void *mem_pool)
{
    os_handle_t mailbox;

    do
    {
        mailbox = rt_malloc(sizeof(os_handle));
        if (mailbox == RT_NULL)
            break;
        if (mem_pool != NULL)
        {
            mailbox->handle = rt_malloc(sizeof(struct rt_mailbox));
            mailbox->flag = RT_OS_ADAPTOR_STATIC;
            if (mailbox)
                rt_mb_init((rt_mailbox_t)mailbox->handle, name, mem_pool, size, RT_IPC_FLAG_FIFO);
        }
        else
        {
            mailbox->handle = (void *)rt_mb_create(name, size, RT_IPC_FLAG_FIFO);
            mailbox->flag = RT_OS_ADAPTOR_DYNAMIC;
        }

        if (mailbox->handle == RT_NULL)
        {
            rt_free(mailbox);
            mailbox = NULL;
        }

    }
    while (0);

    return (os_mailbox_t)mailbox;
}

__ROM_USED rt_err_t os_mailbox_delete_int(os_mailbox_t mailbox)
{
    rt_err_t err = RT_ERROR;
    os_handle_t q_cb = (os_handle_t)mailbox;
    if (q_cb->flag == RT_OS_ADAPTOR_STATIC)
        err = rt_mb_detach((rt_mailbox_t)(q_cb->handle));
    else if (q_cb->flag == RT_OS_ADAPTOR_DYNAMIC)
        err = rt_mb_delete((rt_mailbox_t)(q_cb->handle));
    rt_free(mailbox);
    return err;
}
#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
