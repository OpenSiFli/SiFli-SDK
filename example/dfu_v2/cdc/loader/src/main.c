/**
 * @file main.c
 * @brief DFU-CDC Application Main Entry (V2)
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>

#include "dfu_v2.h"

#ifdef PKG_USING_LITTLEVGL2RTT
#include "dfu_ui.h"
#endif

#define LOG_TAG "dfu.cdc"
#include <ulog.h>

/*============================================================================
 * Constants
 *============================================================================*/

/* USB connection state messages from cdc_acm_init.c */
#define USB_CDC_DFU_CONNECTED      10
#define USB_CDC_DFU_DISCONNECTED   11

#define UI_THREAD_STACK         8192
#define UI_THREAD_PRIO          25

/*============================================================================
 * Global: mailbox for cdc_acm_init.c to send USB state messages
 *============================================================================*/

rt_mailbox_t g_bt_app_mb = RT_NULL;

/*============================================================================
 * UI Thread
 *============================================================================*/

#ifdef PKG_USING_LITTLEVGL2RTT
static struct rt_thread s_ui_thread;
static uint8_t s_ui_stack[UI_THREAD_STACK];

static void ui_thread_entry(void *param)
{
    dfu_ui_task(param);
}
#endif

/*============================================================================
 * DFU V2 Event Callback → UI
 *============================================================================*/

static int on_dfu_event(const dfu_event_param_t *param, void *user_data)
{
    (void)user_data;

    switch (param->event)
    {
    case DFU_EVT_STATE_CHANGED:
        LOG_I("state: %d -> %d", param->state.old_state, param->state.new_state);
        break;

    case DFU_EVT_PROGRESS:
#ifdef PKG_USING_LITTLEVGL2RTT
        {
            char buf[16];
            rt_snprintf(buf, sizeof(buf), "%d", param->progress.percent);
            dfu_ui_update_message(UI_MSG_UPDATE_PROGRESS, buf);
        }
#endif
        LOG_D("progress: %d%%", param->progress.percent);
        break;

    case DFU_EVT_IMG_COMPLETE:
        LOG_I("image %d complete", param->img_complete.img_id);
        break;

    case DFU_EVT_ALL_COMPLETE:
        LOG_I("all images complete");
#ifdef PKG_USING_LITTLEVGL2RTT
        dfu_ui_update_message(UI_MSG_UPDATE_PROGRESS_COLOR, PROGRESS_COLOR_SUCCESS);
        dfu_ui_update_message(UI_MSG_SHOW_SUCCESS_POPUP, RT_NULL);
#endif
        break;

    case DFU_EVT_ERROR:
        LOG_E("DFU error: %d", param->err.error);
#ifdef PKG_USING_LITTLEVGL2RTT
        dfu_ui_update_message(UI_MSG_UPDATE_PROGRESS_COLOR, PROGRESS_COLOR_ERROR);
        dfu_ui_update_message(UI_MSG_SHOW_FAILURE_POPUP, "DFU Error");
#endif
        break;

    case DFU_EVT_ABORTED:
        LOG_W("DFU aborted");
        break;

    default:
        break;
    }

    return 0;
}

/*============================================================================
 * File System Mount Table
 *============================================================================*/

#if defined(RT_USING_DFS_MNTTABLE)
#include "dfs_posix.h"
#include "flash_map.h"
const struct dfs_mount_tbl mount_table[] = {FS_MOUNT_TABLE};
#endif

/*============================================================================
 * main
 *============================================================================*/

int main(void)
{
    LOG_I("=== DFU-CDC V2 Application Start ===");

    /*--- 1. Initialize DFU V2 middleware ---*/
    dfu_config_t cfg = {
        .callback  = on_dfu_event,
        .user_data = RT_NULL,
    };

    if (dfu_init(&cfg) != 0)
    {
        LOG_E("dfu_init failed, USB will still run but DFU transfers disabled");
        /* Don't return — continue to USB event loop so CDC ping works
         * and we can diagnose the issue via serial console */
    }
    else
    {
        LOG_I("DFU V2 middleware initialized");
    }

    /* Expose mailbox to cdc_acm_init.c for USB connection state messages */
    g_bt_app_mb = dfu_get_mailbox();

    /* If middleware init failed, mailbox may be NULL — create a fallback */
    if (g_bt_app_mb == RT_NULL)
    {
        static struct rt_mailbox s_fallback_mb;
        static rt_ubase_t s_fallback_mb_pool[16];
        rt_mb_init(&s_fallback_mb, "dfu_fb", s_fallback_mb_pool,
                   sizeof(s_fallback_mb_pool) / sizeof(s_fallback_mb_pool[0]),
                   RT_IPC_FLAG_FIFO);
        g_bt_app_mb = &s_fallback_mb;
    }

    /*--- 2. Check and execute pending installation ---*/
    dfu_check_install();

    /*--- 3. UI initialization ---*/
#ifdef PKG_USING_LITTLEVGL2RTT
    {
        rt_err_t ret = rt_thread_init(&s_ui_thread, "dfu_ui",
                                      ui_thread_entry, RT_NULL,
                                      s_ui_stack, sizeof(s_ui_stack),
                                      UI_THREAD_PRIO, 20);
        if (ret == RT_EOK)
        {
            rt_thread_startup(&s_ui_thread);
            LOG_I("UI thread started");
        }

        dfu_ui_update_message(UI_MSG_UPDATE_USB, UI_MSG_DATA_USB_DISCONNECTED);
        dfu_ui_update_message(UI_MSG_UPDATE_PROGRESS, "0");
    }
#endif

    LOG_I("=== DFU-CDC V2 Ready, waiting for USB connection ===");

    /*--- 4. Main message loop ---*/
    rt_mailbox_t mb = g_bt_app_mb;
    rt_ubase_t msg;

    while (1)
    {
        if (rt_mb_recv(mb, &msg, RT_WAITING_FOREVER) != RT_EOK)
            continue;

        /* Let V2 middleware handle its messages (DFU_MSG_CDC_DATA, etc.) */
        if (dfu_process(msg) == 0)
            continue;

        /* Middleware didn't recognize it — handle USB connection state */
        switch (msg)
        {
        case USB_CDC_DFU_CONNECTED:
            LOG_I("USB CDC connected");
#ifdef PKG_USING_LITTLEVGL2RTT
            dfu_ui_update_message(UI_MSG_UPDATE_USB, UI_MSG_DATA_USB_CONNECTED);
            dfu_ui_update_message(UI_MSG_UPDATE_SOURCE, UI_MSG_DATA_SOURCE_USB);
#endif
            break;

        case USB_CDC_DFU_DISCONNECTED:
            LOG_I("USB CDC disconnected");
#ifdef PKG_USING_LITTLEVGL2RTT
            dfu_ui_update_message(UI_MSG_UPDATE_USB, UI_MSG_DATA_USB_DISCONNECTED);
            dfu_ui_update_message(UI_MSG_UPDATE_SOURCE, UI_MSG_DATA_SOURCE_NONE);
#endif
            break;

        default:
            LOG_D("Unknown message: %lu", (unsigned long)msg);
            break;
        }
    }

    return 0;
}

/*============================================================================
 * Finsh Debug Commands
 *============================================================================*/

#ifdef RT_USING_FINSH
#include <finsh.h>

extern bool cdc_acm_is_ready(void);

static void cdc_dfu_info(int argc, char **argv)
{
    rt_kprintf("\n=== CDC DFU V2 Info ===\n");
    rt_kprintf("USB Ready: %s\n", cdc_acm_is_ready() ? "Yes" : "No");

    uint32_t received = 0, total = 0;
    if (dfu_get_progress(&received, &total) == 0 && total > 0)
    {
        rt_kprintf("Progress: %d%% (%u/%u bytes)\n",
                   (int)(received * 100 / total), received, total);
    }
    else
    {
        rt_kprintf("Progress: idle\n");
    }

    rt_kprintf("====================\n");
}
MSH_CMD_EXPORT(cdc_dfu_info, Show CDC DFU V2 information);

#endif