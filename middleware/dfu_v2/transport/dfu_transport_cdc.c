/**
 * @file dfu_transport_cdc.c
 * @brief DFU V2 CDC (USB) Transport Implementation
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dfu_transport_cdc.h"
#include "dfu_v2.h"             /* DFU_MSG_CDC_DATA */

#include <rtthread.h>
#include <rtdevice.h>           /* rt_ringbuffer */
#include <string.h>

#define DBG_TAG    "dfu.cdc"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

/*============================================================================
 * Configuration
 *============================================================================*/

#ifndef DFU_V2_CDC_RINGBUF_SIZE
#define DFU_V2_CDC_RINGBUF_SIZE    4096
#endif

/*============================================================================
 * Module Context
 *============================================================================*/

typedef struct
{
    bool                initialized;

    /* Ring buffer for ISR → main-loop data handoff */
    struct rt_ringbuffer rb;
    uint8_t             rb_pool[DFU_V2_CDC_RINGBUF_SIZE];

    /* APP layer mailbox — ISR posts DFU_MSG_CDC_DATA here */
    rt_mailbox_t        mb;

    /* Push Transport vtable instance */
    dfu_push_transport_t transport;

} cdc_ctx_t;

static cdc_ctx_t g_cdc;

/*============================================================================
 * Hardware Transmit — weak default, overridden by cdc_acm_init.c
 *============================================================================*/

__attribute__((weak))
int dfu_cdc_hw_transmit(const uint8_t *data, uint16_t len)
{
    (void)data;
    (void)len;
    LOG_W("dfu_cdc_hw_transmit not implemented — link cdc_acm_init.c");
    return -1;
}

/*============================================================================
 * Push Transport send() — calls CherryUSB hw_transmit
 *============================================================================*/

static int cdc_send(const uint8_t *data, uint16_t len)
{
    if (!g_cdc.initialized)
        return -1;

    if (!data || len == 0)
        return -1;

    return dfu_cdc_hw_transmit(data, len);
}

/*============================================================================
 * Public API: Lifecycle
 *============================================================================*/

int dfu_transport_cdc_init(rt_mailbox_t mb)
{
    if (g_cdc.initialized)
    {
        LOG_W("CDC transport already initialized");
        return 0;
    }

    if (!mb)
    {
        LOG_E("mailbox handle is NULL");
        return -1;
    }

    memset(&g_cdc, 0, sizeof(g_cdc));
    g_cdc.mb = mb;

    /* Initialize ring buffer */
    rt_ringbuffer_init(&g_cdc.rb, g_cdc.rb_pool, sizeof(g_cdc.rb_pool));

    /* Set up Push Transport vtable */
    g_cdc.transport.send = cdc_send;
    g_cdc.initialized = true;

    LOG_I("CDC transport initialized (ringbuf=%u bytes, hw=CherryUSB)",
          DFU_V2_CDC_RINGBUF_SIZE);
    return 0;
}

void dfu_transport_cdc_deinit(void)
{
    if (!g_cdc.initialized)
        return;

    rt_ringbuffer_reset(&g_cdc.rb);
    g_cdc.initialized = false;
    LOG_I("CDC transport deinitialized");
}

/*============================================================================
 * ISR-side: Raw Data Feed
 *============================================================================*/

int dfu_transport_cdc_feed_raw(const uint8_t *data, uint32_t len)
{
    if (!g_cdc.initialized || !data || len == 0)
        return 0;

    rt_size_t put = rt_ringbuffer_put(&g_cdc.rb, data, len);
    if (put < len)
    {
        rt_kprintf("[dfu.cdc] ring buffer overflow, dropped %u bytes\n",
                   (unsigned)(len - put));
    }

    /* Notify APP main loop — non-blocking.
     * If mailbox is full the message is lost, but the next ISR
     * will post again. Main loop always drains the ring buffer
     * fully on each DFU_MSG_CDC_DATA, so no data is lost. */
    if (g_cdc.mb)
        rt_mb_send(g_cdc.mb, DFU_MSG_CDC_DATA);

    return (int)put;
}

/*============================================================================
 * APP-side: Receive Data Retrieval
 *============================================================================*/

int dfu_transport_cdc_get_data(uint8_t *buf, uint32_t max_len)
{
    if (!buf || max_len == 0)
        return -1;

    if (!g_cdc.initialized)
        return -1;

    rt_size_t n = rt_ringbuffer_get(&g_cdc.rb, buf, max_len);
    return (int)n;
}

/*============================================================================
 * Transport Instance
 *============================================================================*/

const dfu_push_transport_t *dfu_transport_cdc_get_instance(void)
{
    if (!g_cdc.initialized)
        return RT_NULL;

    return &g_cdc.transport;
}
