/**
 * @file dfu_v2.c
 * @brief DFU V2 — Middleware API Implementation
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dfu_v2.h"

/* Mode headers (conditional) */
#ifdef DFU_V2_USE_HOST_MODE
#include "mode/dfu_mode_host.h"
#endif
#ifdef DFU_V2_USE_DEVICE_MODE
#include "mode/dfu_mode_device.h"
#endif

/* Transport headers (conditional) */
#ifdef DFU_V2_USE_CDC_TRANSPORT
#include "transport/dfu_transport_cdc.h"
#endif
#ifdef DFU_V2_USE_BLE_TRANSPORT
#include "transport/dfu_transport_ble.h"
#endif
#ifdef DFU_V2_USE_PAN_TRANSPORT
#include "transport/dfu_transport_pan.h"
#endif

/* Security primitives for ctrl_packet decryption */
#ifdef DFU_V2_USE_BLE_TRANSPORT
#include "dfu_sec.h"
#endif

#include <rtthread.h>
#include <string.h>
#include <board.h>

#include "dfu_fwinfo.h"

/* DFU force mode enum (from SDK middleware_dfu/dfu.h, duplicated here
 * to avoid pulling in the entire old DFU header which conflicts with V2) */
#ifndef DFU_FORCE_MODE_REBOOT_TO_USER
#define DFU_FORCE_MODE_REBOOT_TO_USER  1
#endif

#define DBG_TAG    "dfu.v2"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

/*============================================================================
 * Configuration
 *============================================================================*/

#ifndef DFU_V2_MB_SIZE
#define DFU_V2_MB_SIZE      32
#endif

#ifndef DFU_RX_BUFSZ
#define DFU_RX_BUFSZ    1024
#endif

/*============================================================================
 * Module Context
 *============================================================================*/

typedef struct
{
    bool                initialized;
    rt_mailbox_t        mb;

    /* Subprogram event callback */
    dfu_event_cb_t   callback;
    void               *user_data;
} dfu_ctx_t;

static dfu_ctx_t g_ctx;

/* Mailbox pool (static allocation) */
static rt_ubase_t      s_mb_pool[DFU_V2_MB_SIZE];
static struct rt_mailbox s_mb_obj;

/*============================================================================
 * Internal: Fire Event Callback
 *============================================================================*/

static void fire_event(const dfu_event_param_t *param)
{
    if (g_ctx.callback)
        g_ctx.callback(param, g_ctx.user_data);
}

/*============================================================================
 * Engine Callbacks → Event Forwarding
 *============================================================================*/

static void on_engine_state_change(dfu_engine_state_t old_state,
                                   dfu_engine_state_t new_state,
                                   dfu_engine_err_t error,
                                   void *user_data)
{
    (void)user_data;
    LOG_I("engine: %d -> %d (err=%d)", old_state, new_state, error);

    dfu_event_param_t evt;
    evt.event = DFU_EVT_STATE_CHANGED;
    evt.state.old_state = old_state;
    evt.state.new_state = new_state;
    evt.state.error     = error;
    fire_event(&evt);

    /* Also post to mailbox for subprogram's main loop awareness */
    if (g_ctx.mb)
        rt_mb_send(g_ctx.mb, DFU_MSG_STATE_CHANGED);
}

static void on_engine_progress(const dfu_progress_t *progress,
                               void *user_data)
{
    (void)user_data;
    LOG_D("progress: %u/%u (%u%%)",
          progress->bytes_received, progress->bytes_total, progress->percent);

    dfu_event_param_t evt;
    evt.event = DFU_EVT_PROGRESS;
    evt.progress.bytes_received = progress->bytes_received;
    evt.progress.bytes_total   = progress->bytes_total;
    evt.progress.percent       = progress->percent;
    fire_event(&evt);

    if (g_ctx.mb)
        rt_mb_send(g_ctx.mb, DFU_MSG_PROGRESS);
}

/*============================================================================
 * Mode Event Callback
 *============================================================================*/

static void on_mode_event(const dfu_mode_event_param_t *param,
                          void *user_data)
{
    (void)user_data;

    dfu_event_param_t evt;
    memset(&evt, 0, sizeof(evt));

    switch (param->event)
    {
    case DFU_MODE_EVT_STARTED:
        LOG_I("upgrade started");
        return; /* no dedicated V2 event for this */

    case DFU_MODE_EVT_PROGRESS:
        LOG_D("mode progress: %u%%", param->progress.percent);
        return; /* engine progress callback covers this */

    case DFU_MODE_EVT_IMG_COMPLETE:
        LOG_I("image %u completed", param->img_id);
        evt.event = DFU_EVT_IMG_COMPLETE;
        evt.img_complete.img_id = param->img_id;
        break;

    case DFU_MODE_EVT_ALL_COMPLETE:
        LOG_I("=== ALL IMAGES COMPLETE ===");
        evt.event = DFU_EVT_ALL_COMPLETE;
        break;

    case DFU_MODE_EVT_ERROR:
        LOG_E("upgrade error: %d", param->error);
        evt.event = DFU_EVT_ERROR;
        evt.err.error = param->error;
        break;

    case DFU_MODE_EVT_ABORTED:
        LOG_W("upgrade aborted");
        evt.event = DFU_EVT_ABORTED;
        break;

    default:
        return;
    }

    fire_event(&evt);
}

/*============================================================================
 * dfu_init
 *============================================================================*/

int dfu_init(const dfu_config_t *cfg)
{
    if (g_ctx.initialized)
    {
        LOG_W("already initialized");
        return 0;
    }

    memset(&g_ctx, 0, sizeof(g_ctx));

    /*--- 1. Save callback ---*/
    if (cfg)
    {
        g_ctx.callback  = cfg->callback;
        g_ctx.user_data = cfg->user_data;
    }

    /*--- 2. Create mailbox (always internal) ---*/
    rt_err_t mb_err = rt_mb_init(&s_mb_obj, "dfu_mb",
                                  s_mb_pool, DFU_V2_MB_SIZE,
                                  RT_IPC_FLAG_FIFO);
    if (mb_err != RT_EOK)
    {
        LOG_E("mailbox creation failed");
        return -1;
    }
    g_ctx.mb = &s_mb_obj;

    /*--- 3. Security init (for ctrl_packet decryption) ---*/
#ifdef DFU_V2_USE_BLE_TRANSPORT
    dfu_sec_init();
#endif

    /*--- 4. Engine ---*/
    dfu_engine_config_t eng_cfg;
    memset(&eng_cfg, 0, sizeof(eng_cfg));
    eng_cfg.on_state_change   = on_engine_state_change;
    eng_cfg.on_progress       = on_engine_progress;
    eng_cfg.user_data         = RT_NULL;
    eng_cfg.progress_interval = DFU_V2_ENGINE_PROGRESS_INTERVAL;  /* default */

    dfu_engine_err_t err = dfu_engine_init(&eng_cfg);
    if (err != DFU_ENGINE_ERR_NONE)
    {
        LOG_E("engine init failed: %d", err);
        return -1;
    }

    /*--- 4. Transport(s) ---*/
#ifdef DFU_V2_USE_CDC_TRANSPORT
    if (dfu_transport_cdc_init(g_ctx.mb) != 0)
    {
        LOG_E("CDC transport init failed");
        dfu_engine_deinit();
        return -1;
    }
#endif

#ifdef DFU_V2_USE_BLE_TRANSPORT
    if (dfu_transport_ble_init(g_ctx.mb) != 0)
    {
        LOG_E("BLE transport init failed");
        dfu_engine_deinit();
        return -1;
    }
#endif

#ifdef DFU_V2_USE_PAN_TRANSPORT
    if (dfu_transport_pan_init() != 0)
    {
        LOG_E("PAN transport init failed");
        dfu_engine_deinit();
        return -1;
    }
#endif

    /*--- 5. Mode(s) ---*/
#ifdef DFU_V2_USE_HOST_MODE
    {
        const dfu_push_transport_t *push_tp = RT_NULL;
        bool is_msg_transport = false;
        const char *tp_name = "unknown";

#ifdef DFU_V2_USE_CDC_TRANSPORT
        push_tp = dfu_transport_cdc_get_instance();
        is_msg_transport = false;
        tp_name = "cdc";
#endif

#ifdef DFU_V2_USE_BLE_TRANSPORT
        if (!push_tp)
        {
            push_tp = dfu_transport_ble_get_instance();
            is_msg_transport = true;
            tp_name = "ble";
        }
#endif

        if (push_tp)
        {
            if (dfu_mode_host_init(push_tp, is_msg_transport, tp_name,
                                   on_mode_event, RT_NULL) != 0)
            {
                LOG_E("host mode init failed");
                dfu_engine_deinit();
                return -1;
            }
        }
        else
        {
            LOG_W("host mode enabled but no push transport");
        }
    }
#endif /* DFU_V2_USE_HOST_MODE */

#ifdef DFU_V2_USE_DEVICE_MODE
    {
        const dfu_pull_transport_t *pull_tp = RT_NULL;

#ifdef DFU_V2_USE_PAN_TRANSPORT
        pull_tp = dfu_transport_pan_get_instance();
#endif

        if (pull_tp)
        {
            if (dfu_mode_device_init(pull_tp, on_mode_event, RT_NULL) != 0)
            {
                LOG_E("device mode init failed");
                dfu_engine_deinit();
                return -1;
            }
        }
        else
        {
            LOG_W("device mode enabled but no pull transport");
        }
    }
#endif /* DFU_V2_USE_DEVICE_MODE */

    /* ❌ No thread creation — subprogram owns the main loop */

    g_ctx.initialized = true;
    LOG_I("DFU V2 initialized (no internal thread)");
    return 0;
}

/*============================================================================
 * dfu_deinit
 *============================================================================*/

void dfu_deinit(void)
{
    if (!g_ctx.initialized)
        return;

#ifdef DFU_V2_USE_DEVICE_MODE
    dfu_mode_device_deinit();
#endif
#ifdef DFU_V2_USE_HOST_MODE
    dfu_mode_host_deinit();
#endif
#ifdef DFU_V2_USE_PAN_TRANSPORT
    dfu_transport_pan_deinit();
#endif
#ifdef DFU_V2_USE_BLE_TRANSPORT
    dfu_transport_ble_deinit();
#endif
#ifdef DFU_V2_USE_CDC_TRANSPORT
    dfu_transport_cdc_deinit();
#endif

    dfu_engine_deinit();

    rt_mb_detach(&s_mb_obj);
    g_ctx.mb = RT_NULL;

    g_ctx.initialized = false;
    LOG_I("DFU V2 deinitialized");
}

/*============================================================================
 * dfu_get_mailbox
 *============================================================================*/

rt_mailbox_t dfu_get_mailbox(void)
{
    return g_ctx.mb;
}

/*============================================================================
 * dfu_process
 *============================================================================*/

int dfu_process(rt_ubase_t msg)
{
    switch ((dfu_msg_t)msg)
    {
    /*--- Host Mode: byte-stream transports ---*/
#ifdef DFU_V2_USE_CDC_TRANSPORT
    case DFU_MSG_CDC_DATA:
    {
        uint8_t buf[DFU_RX_BUFSZ];
        int n;
        do {
            n = dfu_transport_cdc_get_data(buf, sizeof(buf));
            if (n > 0)
            {
#ifdef DFU_V2_USE_HOST_MODE
                dfu_mode_host_feed_data(buf, n);
#endif
            }
        } while (n > 0);
        return 0;
    }
#endif

    /* UART transport: not yet implemented
#ifdef DFU_V2_USE_UART_TRANSPORT
    case DFU_MSG_UART_DATA:
    {
        uint8_t buf[DFU_RX_BUFSZ];
        int n;
        do {
            n = dfu_transport_uart_get_data(buf, sizeof(buf));
            if (n > 0)
            {
#ifdef DFU_V2_USE_HOST_MODE
                dfu_mode_host_feed_data(buf, n);
#endif
            }
        } while (n > 0);
        return 0;
    }
#endif
    */

    /*--- Host Mode: message-based transports ---*/
#ifdef DFU_V2_USE_BLE_TRANSPORT
    case DFU_MSG_BLE_DATA:
    {
#ifdef DFU_V2_USE_HOST_MODE
        return dfu_transport_ble_process();  /* -1 when queue empty, 0 when processed */
#else
        return 0;
#endif
    }

    case DFU_MSG_BLE_DISCONNECTED:
    {
        /*
         * BLE serial channel was closed (peer disconnected or link lost).
         * Reset Engine to IDLE so a reconnecting host can start a fresh
         * INIT_REQ. FlashDB progress is preserved for resume.
         */
        LOG_W("BLE disconnected — resetting engine session for reconnect");
#ifdef DFU_V2_USE_HOST_MODE
        dfu_engine_reset_session();
#endif
        return 0;
    }
#endif

    /*--- Abort ---*/
    case DFU_MSG_ABORT:
        LOG_W("abort requested");
        dfu_engine_abort();
        return 0;

    /*--- Engine status (logged in callbacks, swallow here) ---*/
    case DFU_MSG_STATE_CHANGED:
    case DFU_MSG_PROGRESS:
        return 0;

    default:
        return -1;  /* not recognized — subprogram should handle */
    }
}

/*============================================================================
 * dfu_check_install
 *============================================================================*/

int dfu_check_install(void)
{
    /*
     * V2 Engine writes firmware data directly to target flash (no staging).
     * No decompression/decryption step exists in V2.
     * This applies to ALL transports: BLE, CDC, PAN.
     *
     * On boot, check if a previous session left stale update flags
     * and clear them — the data is already in place or the session
     * was interrupted (in which case stale flags must be removed).
     */
    LOG_I("check_install: checking for pending firmware...");

    struct dfu_fw_info info;
    bool has_pending = false;

    for (int i = 0; i < DFU_MAX_FW_FILES; i++)
    {
        if (dfu_fwinfo_get(i, &info) == 0 &&
            info.needs_update == 1 &&
            info.magic == DFU_FW_MAGIC)
        {
            has_pending = true;
            LOG_I("check_install: found pending entry [%d] %s", i, info.name);
        }
    }

    if (has_pending)
    {
        LOG_I("check_install: clearing stale update flags");
        dfu_fwinfo_clear();
    }

    LOG_I("check_install: done, continuing to DFU main loop");
    return 0;
}

/*============================================================================
 * dfu_download
 *============================================================================*/

int dfu_download(const dfu_download_req_t *req)
{
#ifdef DFU_V2_USE_DEVICE_MODE
    if (!g_ctx.initialized)
        return -1;

    if (!req || !req->files || req->file_count <= 0)
        return -1;

    LOG_I("download: %d file(s)", req->file_count);

    for (int i = 0; i < req->file_count; i++)
    {
        const dfu_file_t *f = &req->files[i];

        if (!f->url || f->file_size == 0)
        {
            LOG_W("download: skip file[%d] (no url or zero size)", i);
            continue;
        }

        LOG_I("download: file[%d] img=%u size=%u addr=0x%08x",
              i, f->img_id, f->file_size, f->flash_addr);

        /*
         * Use Device Mode to download this file:
         *   1. Begin engine session (target flash addr, size, crc)
         *   2. Pull Transport opens HTTP connection
         *   3. Loop: read chunk → engine write_data → progress callback
         *   4. Finalize session (CRC verify)
         *
         * dfu_mode_device_start() encapsulates all of this as a
         * blocking call.
         */
        dfu_file_info_t fi;
        memset(&fi, 0, sizeof(fi));
        fi.img_count              = 1;
        fi.total_length           = f->file_size;
        fi.file_crc               = f->file_crc;
        fi.images[0].img_id       = f->img_id;
        fi.images[0].flash_addr   = f->flash_addr;
        fi.images[0].img_length   = f->file_size;
        fi.images[0].flash_size   = f->region_size;
        fi.images[0].img_crc      = f->file_crc;

        int ret = dfu_mode_device_start(&fi, f->url);
        if (ret != 0)
        {
            LOG_E("download: file[%d] failed: %d", i, ret);
            return ret;
        }

        LOG_I("download: file[%d] done", i);
    }

    LOG_I("download: all files complete");
    return 0;
#else
    LOG_E("download: device mode not enabled");
    return -1;
#endif
}

/*============================================================================
 * dfu_set_install_flag
 *============================================================================*/

int dfu_set_install_flag(void)
{
    /*
     * Set bootloader flags so next reboot applies the update.
     * V2 uses the same mechanism for all transports:
     *   1. dfu_fwinfo_set_update_flags() marks entries in flash
     *   2. RTC backup register tells bootloader to jump to user FW after
     *
     * PAN transport has its own flag mechanism for server-side coordination,
     * so it gets a dedicated path.
     */
    LOG_I("set_install_flag: setting bootloader flag");

#ifdef DFU_V2_USE_PAN_TRANSPORT
    /* PAN path: use PAN middleware's flag mechanism */
    extern int dfu_pan_set_update_flags(void);
    int ret = dfu_pan_set_update_flags();
    if (ret != 0)
    {
        LOG_E("set_install_flag: dfu_pan_set_update_flags failed: %d", ret);
        return -1;
    }
#else
    /* BLE / CDC path: V2 fwinfo + RTC backup */
    dfu_fwinfo_set_update_flags();
    HAL_Set_backup(RTC_BAKCUP_OTA_FORCE_MODE, DFU_FORCE_MODE_REBOOT_TO_USER);
#endif

    LOG_I("set_install_flag: done");
    return 0;
}

/*============================================================================
 * dfu_abort
 *============================================================================*/

int dfu_abort(void)
{
    if (!g_ctx.mb)
        return -1;

    rt_err_t ret = rt_mb_send(g_ctx.mb, DFU_MSG_ABORT);
    return (ret == RT_EOK) ? 0 : -1;
}

/*============================================================================
 * dfu_get_progress
 *============================================================================*/

int dfu_get_progress(uint32_t *received, uint32_t *total)
{
    dfu_engine_status_t st;
    if (dfu_engine_get_status(&st) != DFU_ENGINE_ERR_NONE)
        return -1;

    if (received) *received = st.progress.bytes_received;
    if (total)    *total    = st.progress.bytes_total;
    return 0;
}