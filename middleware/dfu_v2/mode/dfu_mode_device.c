/**
 * @file dfu_mode_device.c
 * @brief DFU V2 Device-Initiated Mode Implementation
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dfu_mode_device.h"
#include "engine/dfu_engine.h"

#include <rtthread.h>
#include <string.h>

#define DBG_TAG    "dfu.dev"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

/*============================================================================
 * Configuration
 *============================================================================*/

/** Read buffer size for transport->read() calls.
 *  Larger = fewer read calls but more stack usage. */
#ifndef DFU_DEV_READ_BUFSZ
#define DFU_DEV_READ_BUFSZ      4096
#endif

/*============================================================================
 * Module Context
 *============================================================================*/

typedef struct
{
    bool                          initialized;
    bool                          busy;

    const dfu_pull_transport_t   *transport;
    dfu_mode_callback_t           callback;
    void                         *user_data;
} device_ctx_t;

static device_ctx_t g_dev;

/*============================================================================
 * Internal Helpers
 *============================================================================*/

/**
 * @brief Notify APP layer via callback
 */
static void notify_app(dfu_mode_event_t event, dfu_engine_err_t error)
{
    if (!g_dev.callback)
        return;

    dfu_mode_event_param_t param;
    memset(&param, 0, sizeof(param));
    param.event = event;
    param.error = error;

    /* Fill progress if relevant */
    if (event == DFU_MODE_EVT_PROGRESS)
    {
        dfu_engine_status_t st;
        if (dfu_engine_get_status(&st) == DFU_ENGINE_ERR_NONE)
            param.progress = st.progress;
    }

    g_dev.callback(&param, g_dev.user_data);
}

/**
 * @brief Run the download + write loop for one image
 *
 * Reads from transport in chunks and writes to Engine until
 * all bytes for the image are transferred or an error occurs.
 *
 * @param download_length  Bytes to download (may be < img_length if resuming)
 * @return DFU_ENGINE_ERR_NONE on success, error code otherwise
 */
static dfu_engine_err_t download_loop(uint32_t download_length)
{
    uint8_t buf[DFU_DEV_READ_BUFSZ];
    uint32_t remaining = download_length;
    dfu_engine_err_t err;

    while (remaining > 0)
    {
        uint32_t to_read = remaining < sizeof(buf) ? remaining : sizeof(buf);

        int n = g_dev.transport->read(buf, to_read);
        if (n < 0)
        {
            LOG_E("transport read failed: %d", n);
            return DFU_ENGINE_ERR_FLASH_READ;  /* reuse as transport error */
        }
        if (n == 0)
        {
            LOG_E("unexpected EOF: remaining=%u", remaining);
            return DFU_ENGINE_ERR_CRC_MISMATCH;  /* data incomplete */
        }

        /* Write to Engine — Engine maintains running_crc internally */
        err = dfu_engine_write_data(buf, n);
        if (err != DFU_ENGINE_ERR_NONE)
        {
            if (err == DFU_ENGINE_ERR_ABORTED)
                LOG_I("abort detected during write");
            else
                LOG_E("write_data failed: %d", err);
            return err;
        }

        remaining -= n;

        /* Periodic progress notification */
        notify_app(DFU_MODE_EVT_PROGRESS, DFU_ENGINE_ERR_NONE);
    }

    return DFU_ENGINE_ERR_NONE;
}

/**
 * @brief Run one complete image session
 *
 * @param file_info   Per-image file info (img_count=1, images[0] = this image)
 * @param url         Download URL
 * @param resume_offset  Byte offset to resume from (0 = fresh start)
 * @param img_idx     Image index in the original multi-image file_info (for logging)
 * @return DFU_ENGINE_ERR_NONE on success
 */
static dfu_engine_err_t run_image_session(const dfu_file_info_t *file_info,
                                          const char *url,
                                          uint32_t resume_offset,
                                          uint8_t img_idx)
{
    dfu_engine_err_t err;
    const dfu_image_info_t *img = &file_info->images[0];

    /* 1. Erase (Engine may skip if resuming and already erased) */
    err = dfu_engine_erase();
    if (err != DFU_ENGINE_ERR_NONE)
    {
        LOG_E("img[%u] erase failed: %d", img_idx, err);
        return err;
    }

    /* 2. Open transport (HTTP GET with optional Range header) */
    int ret = g_dev.transport->open(url, resume_offset);
    if (ret < 0)
    {
        LOG_E("img[%u] transport open failed: %d", img_idx, ret);
        return DFU_ENGINE_ERR_FLASH_READ;
    }

    /* 3. Download + write loop */
    uint32_t download_length = img->img_length - resume_offset;
    err = download_loop(download_length);

    /* Always close transport after download (success or failure) */
    g_dev.transport->close();

    if (err != DFU_ENGINE_ERR_NONE)
    {
        LOG_E("img[%u] download failed: %d", img_idx, err);
        return err;
    }

    /* 4. Verify — pass per-image CRC from file_info (from server metadata) */
    err = dfu_engine_verify(img->img_crc);
    if (err != DFU_ENGINE_ERR_NONE)
    {
        LOG_E("img[%u] verify failed: %d", img_idx, err);
        return err;
    }

    /* 5. Commit */
    err = dfu_engine_commit();
    if (err != DFU_ENGINE_ERR_NONE)
    {
        LOG_E("img[%u] commit failed: %d", img_idx, err);
        return err;
    }

    LOG_I("img[%u] (id=%u) completed, crc=0x%08x",
          img_idx, img->img_id, img->img_crc);
    return DFU_ENGINE_ERR_NONE;
}

/*============================================================================
 * Public API: Lifecycle
 *============================================================================*/

int dfu_mode_device_init(const dfu_pull_transport_t *transport,
                         dfu_mode_callback_t callback,
                         void *user_data)
{
    if (!transport || !transport->open || !transport->read || !transport->close)
    {
        LOG_E("invalid transport vtable");
        return -1;
    }

    memset(&g_dev, 0, sizeof(g_dev));
    g_dev.transport = transport;
    g_dev.callback  = callback;
    g_dev.user_data = user_data;
    g_dev.initialized = true;

    LOG_I("device mode initialized");
    return 0;
}

void dfu_mode_device_deinit(void)
{
    if (!g_dev.initialized)
        return;

    if (g_dev.busy)
    {
        dfu_engine_abort();
        g_dev.transport->close();
        g_dev.busy = false;
    }

    g_dev.initialized = false;
    LOG_I("device mode deinitialized");
}

/*============================================================================
 * Public API: Start Upgrade
 *============================================================================*/

int dfu_mode_device_start(const dfu_file_info_t *file_info,
                          const char *url)
{
    if (!g_dev.initialized)
    {
        LOG_E("not initialized");
        return -1;
    }

    if (!file_info || !url || file_info->img_count == 0)
    {
        LOG_E("invalid parameters");
        return -1;
    }

    if (g_dev.busy)
    {
        LOG_E("upgrade already in progress");
        return -1;
    }

    g_dev.busy = true;
    notify_app(DFU_MODE_EVT_STARTED, DFU_ENGINE_ERR_NONE);

    dfu_engine_err_t err;

    /* Process each image sequentially */
    for (uint8_t i = 0; i < file_info->img_count; i++)
    {
        const dfu_image_info_t *img = &file_info->images[i];
        LOG_I("starting img[%u]: id=%u, length=%u, flash=0x%08x",
              i, img->img_id, img->img_length, img->flash_addr);

        /* Build per-image file_info for Engine (img_count=1) */
        dfu_file_info_t per_img;
        memset(&per_img, 0, sizeof(per_img));
        per_img.total_length  = img->img_length;
        per_img.total_packets = (file_info->block_size > 0)
            ? (img->img_length + file_info->block_size - 1) / file_info->block_size
            : 0;
        per_img.file_crc      = file_info->file_crc;  /* for resume matching */
        per_img.hw_version    = file_info->hw_version;
        per_img.sdk_version   = file_info->sdk_version;
        per_img.fw_version    = file_info->fw_version;
        memcpy(per_img.fw_key, file_info->fw_key, DFU_KEY_SIZE);
        per_img.dfu_id        = file_info->dfu_id;
        per_img.block_size    = file_info->block_size;
        per_img.img_count     = 1;
        memcpy(&per_img.images[0], img, sizeof(dfu_image_info_t));

        /* Begin session — checks FlashDB for resume */
        dfu_resume_info_t resume_info;
        err = dfu_engine_begin_session(&per_img, "device-initiated", "pan",
                                       &resume_info);
        if (err != DFU_ENGINE_ERR_NONE)
        {
            LOG_E("img[%u] begin_session failed: %d", i, err);
            notify_app(DFU_MODE_EVT_ERROR, err);
            g_dev.busy = false;
            return -1;
        }

        uint32_t resume_offset = 0;
        if (resume_info.can_resume)
        {
            resume_offset = resume_info.completed_bytes;
            LOG_I("img[%u] resuming from byte %u (packets=%u)",
                  i, resume_offset, resume_info.completed_packets);
        }

        /* Run the complete image session */
        err = run_image_session(&per_img, url, resume_offset, i);
        if (err != DFU_ENGINE_ERR_NONE)
        {
            if (err == DFU_ENGINE_ERR_ABORTED)
                notify_app(DFU_MODE_EVT_ABORTED, err);
            else
                notify_app(DFU_MODE_EVT_ERROR, err);
            g_dev.busy = false;
            return -1;
        }

        /* Notify per-image completion */
        if (g_dev.callback)
        {
            dfu_mode_event_param_t param;
            memset(&param, 0, sizeof(param));
            param.event  = DFU_MODE_EVT_IMG_COMPLETE;
            param.img_id = img->img_id;
            g_dev.callback(&param, g_dev.user_data);
        }
    }

    /* All images done */
    notify_app(DFU_MODE_EVT_ALL_COMPLETE, DFU_ENGINE_ERR_NONE);
    g_dev.busy = false;

    LOG_I("device upgrade completed successfully (%u images)",
          file_info->img_count);
    return 0;
}

/*============================================================================
 * Public API: Status Query
 *============================================================================*/

bool dfu_mode_device_is_busy(void)
{
    return g_dev.busy;
}
