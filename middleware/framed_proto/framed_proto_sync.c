/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file framed_proto_sync.c
 * @brief Link supervision of the framed message channel.
 *
 * A transfer can go quiet while the peer is still connected: the phone stops
 * sending, a reassembly buffer never fills up, a response never comes back.
 * This file owns the single one-shot timer that watches for that and tells the
 * product which kind of transfer went silent. What to do about it - ask for a
 * retransmission, report the loss upward, drop the session - is the product's
 * decision, not this file's.
 *
 * Channel housekeeping only: no storage concept and no product type appears
 * here, which is why screen cast and firmware update can share it.
 */

#include <stdint.h>

#include <rtthread.h>
#include <rthw.h>
#include "os_adaptor.h"

#include "framed_proto.h"

#define LOG_TAG "FRMPROTO"
#include "log.h"

OS_TIMER_DECLAR(g_framed_proto_sync_timer);

static uint8_t s_sync_timer_on;
static framed_proto_sync_timeout_cb_t s_sync_timeout_cb;

/* Retire a running timer. Safe to call when none is armed. */
static void framed_proto_sync_retire(void)
{
    rt_base_t level = rt_hw_interrupt_disable();

    if (s_sync_timer_on)
    {
        s_sync_timer_on = 0;
        os_timer_stop(g_framed_proto_sync_timer);
        os_timer_delete(g_framed_proto_sync_timer);
    }

    rt_hw_interrupt_enable(level);
}

static void framed_proto_sync_fire(uint8_t sync_type)
{

    framed_proto_sync_retire();

    if (s_sync_timeout_cb)
    {
        s_sync_timeout_cb(sync_type);
    }
}

/* One entry point per supervised transfer kind, because os_timer_create() ties
 * a callback to the timer object rather than to its start call. */
static void framed_proto_sync_download_cb(void *parameter)
{
    (void)parameter;
    framed_proto_sync_fire(FRAMED_PROTO_SYNC_DOWNLOAD);
}

static void framed_proto_sync_rsp_cb(void *parameter)
{
    (void)parameter;
    framed_proto_sync_fire(FRAMED_PROTO_SYNC_RSP);
}

static void framed_proto_sync_continual_file_cb(void *parameter)
{
    (void)parameter;
    framed_proto_sync_fire(FRAMED_PROTO_SYNC_CONTINUAL_FILE);
}

static void framed_proto_sync_continual_file_lost_cb(void *parameter)
{
    (void)parameter;
    framed_proto_sync_fire(FRAMED_PROTO_SYNC_CONTINUAL_FILE_LOST);
}

void framed_proto_sync_set_timeout_cb(framed_proto_sync_timeout_cb_t cb)
{
    s_sync_timeout_cb = cb;
}

uint8_t framed_proto_sync_is_on(void)
{
    return s_sync_timer_on;
}

void framed_proto_sync_start(uint8_t sync_type)
{
    uint32_t timeout;
    os_timer_func_t handler;
    rt_base_t level;

    switch (sync_type)
    {
    case FRAMED_PROTO_SYNC_DOWNLOAD:
    {
        handler = framed_proto_sync_download_cb;
        timeout = FRAMED_PROTO_SYNC_FILE_DOWNLOAD_TIMEOUT;
        break;
    }
    case FRAMED_PROTO_SYNC_RSP:
    {
        handler = framed_proto_sync_rsp_cb;
        timeout = FRAMED_PROTO_SYNC_FILE_DOWNLOAD_TIMEOUT;
        break;
    }
    case FRAMED_PROTO_SYNC_CONTINUAL_FILE:
    {
        handler = framed_proto_sync_continual_file_cb;
        timeout = FRAMED_PROTO_SYNC_CONTINUAL_FILE_TIMEOUT;
        break;
    }
    case FRAMED_PROTO_SYNC_CONTINUAL_FILE_LOST:
    {
        handler = framed_proto_sync_continual_file_lost_cb;
        timeout = FRAMED_PROTO_SYNC_CONTINUAL_FILE_LOST_TIMEOUT;
        break;
    }
    default:
    {
        LOG_E("framed_proto_sync_start, unknown type %d", sync_type);
        return;
    }
    }

    level = rt_hw_interrupt_disable();

    if (s_sync_timer_on)
    {
        rt_hw_interrupt_enable(level);
        LOG_I("framed_proto_sync_start, timer already start");
        return;
    }

    s_sync_timer_on = 1;

    os_timer_create(g_framed_proto_sync_timer, handler, NULL,
                    OS_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    os_timer_start(g_framed_proto_sync_timer, timeout);

    rt_hw_interrupt_enable(level);
}

void framed_proto_sync_end(void)
{
    framed_proto_sync_retire();
}
