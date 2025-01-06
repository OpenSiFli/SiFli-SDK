/**
  ******************************************************************************
  * @file   os_adaptor_rtthread_internal.h
  * @author Sifli software development team
  * @brief Header file - OS adaptor internal interface with RTThread OS.
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

#ifndef __OS_ADAPTOR_RTTHREAD_INTERNAL_H
#define __OS_ADAPTOR_RTTHREAD_INTERNAL_H

#define RT_OS_ADAPTOR_STATIC 0
#define RT_OS_ADAPTOR_DYNAMIC 1

typedef struct
{
    void *handle;
    uint8_t flag;
} os_handle;


typedef os_handle *os_handle_t;


extern os_thread_t os_thread_create_int(const char *name, os_thread_func_t entry, void *parameter,
                                        void *stack_memory, uint32_t stack_size, os_priority_t priority,
                                        uint32_t tick);
extern rt_err_t os_thread_delete(os_thread_t thread);
extern os_message_queue_t os_message_queue_create_int(const char *name, uint32_t max_count,
        uint32_t msg_size, void *mem_pool, uint32_t pool_size);
extern rt_err_t os_message_delele_int(os_message_queue_t queue);
extern os_mailbox_t os_mailbox_create_int(const char *name, uint32_t size, void *mem_pool);
extern rt_err_t os_mailbox_delete_int(os_mailbox_t mailbox);

rt_err_t rt_timer_start_int(rt_timer_t timer_id, uint32_t duration);

#endif // __OS_ADAPTOR_RTTHREAD_INTERNAL_H

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
