/**
  ******************************************************************************
  * @file   profiler.c
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

#include <stdio.h>
#include <stdbool.h>
#include <rtthread.h>
#include <SEGGER_RTT.h>
#include <profiler.h>
#include <string.h>
#include "board.h"
#include "mem_section.h"



/* By default, when there is no shell, all events are profiled. */
#ifndef CONFIG_SHELL
    uint32_t profiler_enabled_events = 0xffffffff;
#endif

static bool protocol_running;
static bool sending_events;
static bool event_desc_send;
static uint16_t processing_start_event_id;
static uint16_t processing_end_event_id;


enum profiler_command
{
    PROFILER_COMMAND_START  = 1,
    PROFILER_COMMAND_STOP   = 2,
    PROFILER_COMMAND_INFO   = 3
};

//TODO: use DWT cycle instead
#define cycle_get_32()    (*(volatile uint32_t *)0xE0001004)

char descr[CONFIG_MAX_NUMBER_OF_CUSTOM_EVENTS]
[CONFIG_MAX_LENGTH_OF_CUSTOM_EVENTS_DESCRIPTIONS];
static char *arg_types_encodings[] =
{
    "u8",  /* u8_t */
    "s8",  /* s8_t */
    "u16", /* u16_t */
    "s16", /* s16_t */
    "u32", /* u32_t */
    "s32", /* s32_t */
    "s",   /* string */
    "t"    /* time */
};

uint8_t profiler_num_events;

ALIGN(4)
static uint8_t buffer_data[CONFIG_PROFILER_DATA_BUFFER_SIZE];
ALIGN(4)
static uint8_t buffer_info[CONFIG_PROFILER_INFO_BUFFER_SIZE];
ALIGN(4)
static uint8_t buffer_commands[CONFIG_PROFILER_COMMAND_BUFFER_SIZE];

static struct rt_thread profiler_thread;

L1_NON_RET_BSS_SECT_BEGIN(profiler_stack)
ALIGN(RT_ALIGN_SIZE)
static uint8_t profiler_stack[CONFIG_PROFILER_STACK_SIZE];
L1_NON_RET_BSS_SECT_END

static void send_system_description(void)
{
    size_t num_bytes_send;

    /* Memory barrier to make sure that data is visible
     * before being accessed
     */
    uint8_t ne = profiler_num_events;

    __DMB();
    char end_line = '\n';

    for (size_t t = 0; t < ne; t++)
    {
        num_bytes_send = SEGGER_RTT_WriteNoLock(
                             CONFIG_PROFILER_RTT_CHANNEL_INFO,
                             descr[t],
                             strlen(descr[t]));
        RT_ASSERT(num_bytes_send > 0);
        num_bytes_send = SEGGER_RTT_WriteNoLock(
                             CONFIG_PROFILER_RTT_CHANNEL_INFO,
                             &end_line,
                             1);
        RT_ASSERT(num_bytes_send > 0);
    }
    num_bytes_send = SEGGER_RTT_WriteNoLock(
                         CONFIG_PROFILER_RTT_CHANNEL_INFO,
                         &end_line,
                         1);
    RT_ASSERT(num_bytes_send > 0);
}

static void profiler_register_up_data_channel(void)
{
    int ret = SEGGER_RTT_ConfigUpBuffer(
                  CONFIG_PROFILER_RTT_CHANNEL_DATA,
                  "Profiler data",
                  buffer_data,
                  CONFIG_PROFILER_DATA_BUFFER_SIZE,
                  SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    RT_ASSERT(ret >= 0);
}

static void profiler_thread_fn(void *parameter)
{
    while (protocol_running)
    {
        uint8_t read_data;
        enum profiler_command command;

        if (SEGGER_RTT_Read(
                    CONFIG_PROFILER_RTT_CHANNEL_COMMANDS,
                    &read_data, sizeof(read_data)))
        {
            command = (enum profiler_command)read_data;
            switch (command)
            {
            case PROFILER_COMMAND_START:
            {
                sending_events = true;
                break;
            }
            case PROFILER_COMMAND_STOP:
                sending_events = false;
                break;
            case PROFILER_COMMAND_INFO:
                send_system_description();
                break;
            default:
                RT_ASSERT(false);
                break;
            }
        }
        rt_thread_delay(500);
    }
    //k_sem_give(&profiler_sem);
}

static void register_execution_tracking_events(void)
{
    const char *labels[] = {"mem_address"};
    enum profiler_arg types[] = {PROFILER_ARG_U32};

    /* Event execution start event after last event. */
    processing_start_event_id = profiler_register_event_type(
                                    "event_processing_start",
                                    labels, types, 1);

    /* Event execution end event. */
    processing_end_event_id = profiler_register_event_type(
                                  "event_processing_end",
                                  labels, types, 1);
}

int profiler_init(void)
{
    rt_err_t err;
    protocol_running = true;
    //if (IS_ENABLED(CONFIG_PROFILER_START_LOGGING_ON_SYSTEM_START)) {
    //  sending_events = true;
    //}
    //sending_events = true;
    int ret;

    register_execution_tracking_events();

    SEGGER_RTT_Init();

    profiler_register_up_data_channel();

    ret = SEGGER_RTT_ConfigUpBuffer(
              CONFIG_PROFILER_RTT_CHANNEL_INFO,
              "Profiler info",
              buffer_info,
              CONFIG_PROFILER_INFO_BUFFER_SIZE,
              SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    RT_ASSERT(ret >= 0);

    ret = SEGGER_RTT_ConfigDownBuffer(
              CONFIG_PROFILER_RTT_CHANNEL_COMMANDS,
              "Profiler command",
              buffer_commands,
              CONFIG_PROFILER_COMMAND_BUFFER_SIZE,
              SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    RT_ASSERT(ret >= 0);

    err = rt_thread_init(&profiler_thread,
                         "profiler",
                         profiler_thread_fn,
                         NULL,
                         profiler_stack,
                         sizeof(profiler_stack),
                         CONFIG_PROFILER_THREAD_PRIORITY, 10);
    rt_thread_startup(&profiler_thread);
    return 0;
}

#if 0
void profiler_term(void)
{
    sending_events = false;
    protocol_running = false;
    k_wakeup(protocol_thread_id);
    k_sem_take(&profiler_sem, K_FOREVER);
}
#endif

const char *profiler_get_event_descr(size_t profiler_event_id)
{
    return descr[profiler_event_id];
}

uint16_t profiler_register_event_type(const char *name, const char **args,
                                      const enum profiler_arg *arg_types,
                                      uint8_t arg_cnt)
{
    /* Lock to make sure that this function can be called
     * from multiple threads
     */
    rt_enter_critical();

    uint8_t ne = profiler_num_events;
    size_t temp = snprintf(descr[ne],
                           CONFIG_MAX_LENGTH_OF_CUSTOM_EVENTS_DESCRIPTIONS,
                           "%s,%d", name, ne);
    size_t pos = temp;

    RT_ASSERT((pos < CONFIG_MAX_LENGTH_OF_CUSTOM_EVENTS_DESCRIPTIONS)
              && (temp > 0));

    for (size_t t = 0; t < arg_cnt; t++)
    {
        temp = snprintf(descr[ne] + pos,
                        CONFIG_MAX_LENGTH_OF_CUSTOM_EVENTS_DESCRIPTIONS - pos,
                        ",%s", arg_types_encodings[arg_types[t]]);
        pos += temp;
        RT_ASSERT(
            (pos < CONFIG_MAX_LENGTH_OF_CUSTOM_EVENTS_DESCRIPTIONS)
            && (temp > 0));
    }

    for (size_t t = 0; t < arg_cnt; t++)
    {
        temp = snprintf(descr[ne] + pos,
                        CONFIG_MAX_LENGTH_OF_CUSTOM_EVENTS_DESCRIPTIONS - pos,
                        ",%s", args[t]);
        pos += temp;
        RT_ASSERT(
            (pos < CONFIG_MAX_LENGTH_OF_CUSTOM_EVENTS_DESCRIPTIONS)
            && (temp > 0));
    }
    /* Memory barrier to make sure that data is visible
     * before being accessed
     */
    __DMB();
    profiler_num_events++;
    rt_exit_critical();

    return ne;
}

void profiler_log_start(struct log_event_buf *buf)
{
    /* Adding one to pointer to make space for event type ID */
    RT_ASSERT(sizeof(uint8_t) <= CONFIG_PROFILER_CUSTOM_EVENT_BUF_LEN);
    buf->payload = buf->payload_start + sizeof(uint8_t);
    profiler_log_encode_u32(buf, cycle_get_32());
}

void profiler_log_encode_u32(struct log_event_buf *buf, uint32_t data)
{
    RT_ASSERT(buf->payload - buf->payload_start + sizeof(data)
              <= CONFIG_PROFILER_CUSTOM_EVENT_BUF_LEN);
    //sys_put_le32(data, buf->payload);
    buf->payload[0] = data & 0xFF;
    buf->payload[1] = (data >> 8) & 0xFF;
    buf->payload[2] = (data >> 16) & 0xFF;
    buf->payload[3] = (data >> 24) & 0xFF;
    buf->payload += sizeof(data);
}

void profiler_log_add_string(struct log_event_buf *buf, const char *s)
{
    uint32_t len;

    len = strlen(s);

    RT_ASSERT(buf->payload - buf->payload_start + sizeof(len) + len
              <= CONFIG_PROFILER_CUSTOM_EVENT_BUF_LEN);

    buf->payload[0] = len & 0xFF;
    buf->payload[1] = (len >> 8) & 0xFF;
    buf->payload[2] = (len >> 16) & 0xFF;
    buf->payload[3] = (len >> 24) & 0xFF;
    memcpy(&buf->payload[4], s, len);
    buf->payload += (sizeof(len) + len);
}

void profiler_log_add_mem_address(struct log_event_buf *buf,
                                  const void *mem_address)
{
    profiler_log_encode_u32(buf, (uint32_t)mem_address);
}

void profiler_log_send(struct log_event_buf *buf, uint16_t event_type_id)
{
    uint32_t mask;
    RT_ASSERT(event_type_id <= UINT8_MAX);

    if (sending_events)
    {
        uint8_t type_id = event_type_id & UINT8_MAX;

        buf->payload_start[0] = type_id;
        uint32_t mask = rt_hw_interrupt_disable();

        uint8_t num_bytes_send = SEGGER_RTT_WriteNoLock(
                                     CONFIG_PROFILER_RTT_CHANNEL_DATA,
                                     buf->payload_start,
                                     buf->payload - buf->payload_start);
        //RT_UNUSED(num_bytes_send);
        rt_hw_interrupt_enable(mask);
        //RT_ASSERT(num_bytes_send > 0);
    }
}

void profiler_log_event_execution(uint16_t event_id, bool start)
{
    struct log_event_buf buf;
    uint16_t log_event_id;

    if (sending_events)
    {
        profiler_log_start(&buf);
        profiler_log_add_mem_address(&buf, (void *)(uint32_t)event_id);
        if (start)
        {
            log_event_id = processing_start_event_id;
        }
        else
        {
            log_event_id = processing_end_event_id;
        }
        profiler_log_send(&buf, log_event_id);
    }
}

