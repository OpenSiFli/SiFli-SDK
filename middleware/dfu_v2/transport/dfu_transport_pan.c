/**
 * @file dfu_transport_pan.c
 * @brief DFU V2 PAN HTTP Transport Implementation
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dfu_transport_pan.h"

#include <rtthread.h>
#include <string.h>
#include <stdio.h>
#include <webclient.h>          /* RT-Thread webclient package */
#include <stdbool.h>
#define DBG_TAG    "dfu.pan"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

/*============================================================================
 * Configuration
 *============================================================================*/

/** HTTP response buffer size for webclient header parsing */
#ifndef DFU_V2_PAN_RESP_BUFSZ
#define DFU_V2_PAN_RESP_BUFSZ      1024
#endif

/** HTTP connection timeout in milliseconds */
#ifndef DFU_V2_PAN_TIMEOUT_MS
#define DFU_V2_PAN_TIMEOUT_MS      30000
#endif

/*============================================================================
 * Module Context
 *============================================================================*/

typedef struct
{
    bool                    initialized;
    struct webclient_session *session;
    dfu_pull_transport_t    transport;
} pan_ctx_t;

static pan_ctx_t g_pan;

/*============================================================================
 * Pull Transport vtable implementations
 *============================================================================*/

/**
 * @brief Open HTTP connection to firmware URL
 *
 * @param target  Full HTTP/HTTPS URL of the firmware file
 * @param offset  Byte offset for resume (0 = from beginning)
 * @return 0 on success, negative on failure
 */
static int pan_open(const char *target, uint32_t offset)
{
    if (!target || target[0] == '\0')
    {
        LOG_E("URL is NULL or empty");
        return -1;
    }

    /* Close previous session if any */
    if (g_pan.session)
    {
        webclient_close(g_pan.session);
        g_pan.session = RT_NULL;
    }

    /* Create webclient session */
    g_pan.session = webclient_session_create(DFU_V2_PAN_RESP_BUFSZ);
    if (!g_pan.session)
    {
        LOG_E("webclient_session_create failed");
        return -1;
    }

    /* Set connection timeout */
    webclient_set_timeout(g_pan.session, DFU_V2_PAN_TIMEOUT_MS);

    /* Add Range header for resume */
    if (offset > 0)
    {
        char range_hdr[64];
        rt_snprintf(range_hdr, sizeof(range_hdr), "Range: bytes=%u-", offset);
        webclient_header_fields_add(g_pan.session, range_hdr);
        LOG_I("resume from offset %u", offset);
    }

    /* Send GET request */
    int status = webclient_get(g_pan.session, target);
    if (status != 200 && status != 206)
    {
        LOG_E("HTTP GET failed: status=%d, url=%s", status, target);
        webclient_close(g_pan.session);
        g_pan.session = RT_NULL;
        return -1;
    }

    LOG_I("HTTP %d, content-length=%d", status,
          webclient_content_length_get(g_pan.session));
    return 0;
}

/**
 * @brief Read data from HTTP stream (blocking)
 *
 * @param buf      Destination buffer
 * @param max_len  Maximum bytes to read
 * @return Positive = bytes read, 0 = EOF, negative = error
 */
static int pan_read(uint8_t *buf, uint32_t max_len)
{
    if (!g_pan.session)
        return -1;

    int n = webclient_read(g_pan.session, buf, max_len);
    if (n < 0)
    {
        LOG_E("webclient_read failed: %d", n);
        return -1;
    }
    return n;   /* 0 = EOF, >0 = bytes read */
}

/**
 * @brief Close HTTP connection
 */
static void pan_close(void)
{
    if (g_pan.session)
    {
        webclient_close(g_pan.session);
        g_pan.session = RT_NULL;
        LOG_I("HTTP session closed");
    }
}

/*============================================================================
 * Public API: Lifecycle
 *============================================================================*/

int dfu_transport_pan_init(void)
{
    if (g_pan.initialized)
        return 0;

    memset(&g_pan, 0, sizeof(g_pan));

    g_pan.transport.open  = pan_open;
    g_pan.transport.read  = pan_read;
    g_pan.transport.close = pan_close;

    g_pan.initialized = true;
    LOG_I("PAN HTTP transport initialized");
    return 0;
}

void dfu_transport_pan_deinit(void)
{
    if (!g_pan.initialized)
        return;

    pan_close();
    g_pan.initialized = false;
    LOG_I("PAN HTTP transport deinitialized");
}

/*============================================================================
 * Public API: Transport Instance
 *============================================================================*/

const dfu_pull_transport_t *dfu_transport_pan_get_instance(void)
{
    if (!g_pan.initialized)
        return RT_NULL;

    return &g_pan.transport;
}
