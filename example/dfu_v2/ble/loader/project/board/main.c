/**
 * @file main.c
 * @brief DFU-BLE V2
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>

/* BLE stack */
#include "bf0_ble_gap.h"
#include "bf0_sibles.h"
#include "bf0_sibles_internal.h"
#include "bf0_sibles_advertising.h"
#include "ble_connection_manager.h"

#ifdef BSP_BLE_SERIAL_TRANSMISSION
#include "bf0_sibles_serial_trans_service.h"
#endif

/* DFU V2 middleware */
#include "dfu_v2.h"
#include "mode/dfu_mode_host.h"

/* UI (optional) */
#ifdef PKG_USING_LITTLEVGL2RTT
#include "dfu_ui.h"
#endif

#define LOG_TAG "dfu.ble"
#include <ulog.h>

/*============================================================================
 * File system mount
 *============================================================================*/

#if defined(RT_USING_DFS_MNTTABLE)
#include "dfs_posix.h"
#include "flash_map.h"
const struct dfs_mount_tbl mount_table[] = {FS_MOUNT_TABLE};
#endif

/*============================================================================
 * Constants
 *============================================================================*/

#define DFU_BLE_ADV_NAME        "SIFLI_DFU"
#define UI_THREAD_STACK         8192
#define UI_THREAD_PRIO          25

/** Reboot delay after all images complete (ms) — let UI show success */
#define REBOOT_DELAY_MS         3000

/**
 * DFU mode global timeout (ms).
 * If no upgrade completes within this time, reboot back to user APP.
 * Prevents device from being stuck in DFU mode forever if the phone
 * disconnects and never reconnects. 5 minutes is generous enough for
 * any BLE OTA to complete, even with retransmissions.
 */
#define DFU_MODE_TIMEOUT_MS     (5 * 60 * 1000)

/**
 * DFU inactivity timeout (ms).
 * If BLE is connected but no DFU data arrives for this long, abort
 * and reboot. Catches the case where the phone connects but the APP
 * crashes or the user navigates away.
 */
#define DFU_INACTIVITY_TIMEOUT_MS   (60 * 1000)

/*============================================================================
 * App environment (minimal — just for BLE connection management)
 *============================================================================*/

typedef struct
{
    uint8_t  is_power_on;
    uint8_t  conn_idx;
    uint16_t mtu;
    rt_mailbox_t mb;            /**< Shared mailbox: BLE events + V2 DFU messages */
    rt_timer_t   global_timer;  /**< DFU mode global timeout */
    rt_timer_t   inactivity_timer; /**< Inactivity timeout (reset on each DFU message) */
    bool         upgrade_complete; /**< Set true on ALL_COMPLETE to suppress timeout reboot */
} dfu_ble_env_t;

static dfu_ble_env_t g_env;

/*============================================================================
 * Forward declarations
 *============================================================================*/

static void dfu_ble_advertising_start(void);

/*============================================================================
 * UI Thread (optional)
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
 * DFU V2 Event Callback → UI + Reboot
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
        /* Reset inactivity timer — data is flowing */
        if (g_env.inactivity_timer)
            rt_timer_start(g_env.inactivity_timer);
        LOG_D("progress: %d%%", param->progress.percent);
        break;

    case DFU_EVT_IMG_COMPLETE:
        LOG_I("image %d complete", param->img_complete.img_id);
        break;

    case DFU_EVT_ALL_COMPLETE:
        g_env.upgrade_complete = true;
        /* Stop timeout timers — we're about to reboot anyway */
        if (g_env.global_timer)
            rt_timer_stop(g_env.global_timer);
        if (g_env.inactivity_timer)
            rt_timer_stop(g_env.inactivity_timer);
        LOG_I("=== All images written, rebooting in %d ms ===", REBOOT_DELAY_MS);
#ifdef PKG_USING_LITTLEVGL2RTT
        dfu_ui_update_message(UI_MSG_UPDATE_PROGRESS_COLOR, PROGRESS_COLOR_SUCCESS);
        dfu_ui_update_message(UI_MSG_SHOW_SUCCESS_POPUP, RT_NULL);
#endif
        rt_thread_mdelay(REBOOT_DELAY_MS);
        HAL_PMU_Reboot();
        break;

    case DFU_EVT_ERROR:
        LOG_E("DFU error: %d", param->err.error);
#ifdef PKG_USING_LITTLEVGL2RTT
        dfu_ui_update_message(UI_MSG_UPDATE_PROGRESS_COLOR, PROGRESS_COLOR_ERROR);
        dfu_ui_update_message(UI_MSG_SHOW_FAILURE_POPUP, "更新失败");
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
 * Timeout Handlers
 *============================================================================*/

/**
 * @brief Global DFU mode timeout — reboot back to user APP
 *
 * Fires once after DFU_MODE_TIMEOUT_MS. If upgrade hasn't completed,
 * the device is stuck — reboot to recover.
 */
static void dfu_global_timeout_handler(void *param)
{
    (void)param;
    if (g_env.upgrade_complete)
        return;  /* ALL_COMPLETE will handle reboot */

    LOG_W("DFU mode global timeout (%d s), rebooting to user APP",
          DFU_MODE_TIMEOUT_MS / 1000);
    HAL_PMU_Reboot();
}

/**
 * @brief Inactivity timeout — no DFU data for too long
 *
 * Fires when no DFU progress message has been received for
 * DFU_INACTIVITY_TIMEOUT_MS. Reboot to avoid hanging.
 */
static void dfu_inactivity_timeout_handler(void *param)
{
    (void)param;
    if (g_env.upgrade_complete)
        return;

    LOG_W("DFU inactivity timeout (%d s), rebooting to user APP",
          DFU_INACTIVITY_TIMEOUT_MS / 1000);
    HAL_PMU_Reboot();
}

/*============================================================================
 * BLE Advertising — simplified for DFU mode
 *============================================================================*/

SIBLES_ADVERTISING_CONTEXT_DECLAR(g_adv_context);

static uint8_t ble_adv_event(uint8_t event, void *context, void *data)
{
    switch (event)
    {
    case SIBLES_ADV_EVT_ADV_STARTED:
    {
        sibles_adv_evt_startted_t *evt = (sibles_adv_evt_startted_t *)data;
        LOG_I("BLE advertising started, status=%d", evt->status);
        break;
    }
    case SIBLES_ADV_EVT_ADV_STOPPED:
    {
        sibles_adv_evt_stopped_t *evt = (sibles_adv_evt_stopped_t *)data;
        LOG_I("BLE advertising stopped, reason=%d", evt->reason);
        break;
    }
    default:
        break;
    }
    return 0;
}

static void dfu_ble_advertising_start(void)
{
    sibles_advertising_para_t para = {0};
    uint8_t ret;

    /* Build device name: SIFLI_DFU-xx-xx-xx-xx-xx-xx */
    char local_name[40] = {0};
    bd_addr_t addr;
    ret = ble_get_public_address(&addr);
    if (ret == HL_ERR_NO_ERROR)
    {
        rt_snprintf(local_name, sizeof(local_name),
                    "%s-%x-%x-%x-%x-%x-%x",
                    DFU_BLE_ADV_NAME,
                    addr.addr[0], addr.addr[1], addr.addr[2],
                    addr.addr[3], addr.addr[4], addr.addr[5]);
    }
    else
    {
        strncpy(local_name, DFU_BLE_ADV_NAME, sizeof(local_name) - 1);
    }

    /* Set GAP device name */
    ble_gap_dev_name_t *dev_name = rt_malloc(sizeof(ble_gap_dev_name_t) + strlen(local_name));
    if (dev_name)
    {
        dev_name->len = strlen(local_name);
        memcpy(dev_name->name, local_name, dev_name->len);
        ble_gap_set_dev_name(dev_name);
        rt_free(dev_name);
    }

    /* Advertising parameters */
    para.own_addr_type = GAPM_STATIC_ADDR;
    para.config.adv_mode = SIBLES_ADV_CONNECT_MODE;
    para.config.mode_config.conn_config.duration = 0;  /* forever */
    para.config.mode_config.conn_config.interval = 0x30;
    para.config.max_tx_pwr = 0x7F;
    para.config.is_auto_restart = 1;  /* restart after disconnect */

    /* Scan response: device name */
    para.rsp_data.completed_name = rt_malloc(strlen(local_name) +
                                             sizeof(sibles_adv_type_name_t));
    if (para.rsp_data.completed_name)
    {
        para.rsp_data.completed_name->name_len = strlen(local_name);
        memcpy(para.rsp_data.completed_name->name, local_name, strlen(local_name));
    }

    /* Manufacturer data (Sifli company ID) */
    uint8_t manu_data[] = {0x20, 0xC4, 0x00, 0x91};
    para.adv_data.manufacturer_data = rt_malloc(sizeof(sibles_adv_type_manufacturer_data_t) +
                                                sizeof(manu_data));
    if (para.adv_data.manufacturer_data)
    {
        para.adv_data.manufacturer_data->company_id = SIG_SIFLI_COMPANY_ID;
        para.adv_data.manufacturer_data->data_len = sizeof(manu_data);
        memcpy(para.adv_data.manufacturer_data->additional_data, manu_data, sizeof(manu_data));
    }

    para.evt_handler = ble_adv_event;

    ret = sibles_advertising_init(g_adv_context, &para);
    if (ret == SIBLES_ADV_NO_ERR)
    {
        sibles_advertising_start(g_adv_context);
        LOG_I("BLE DFU advertising: %s", local_name);
    }
    else
    {
        LOG_E("sibles_advertising_init failed: %d", ret);
    }

    /* Free temp allocations (advertising copies internally) */
    if (para.rsp_data.completed_name)
        rt_free(para.rsp_data.completed_name);
    if (para.adv_data.manufacturer_data)
        rt_free(para.adv_data.manufacturer_data);
}

/*============================================================================
 * BLE Event Handler (registered statically)
 *============================================================================*/

int ble_dfu_event_handler(uint16_t event_id, uint8_t *data, uint16_t len,
                          uint32_t context)
{
    switch (event_id)
    {
    case BLE_POWER_ON_IND:
    {
        if (g_env.mb)
            rt_mb_send(g_env.mb, BLE_POWER_ON_IND);
        break;
    }

    case BLE_GAP_CONNECTED_IND:
    {
        ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
        g_env.conn_idx = ind->conn_idx;
        LOG_I("BLE connected: peer(%x-%x-%x-%x-%x-%x)",
              ind->peer_addr.addr[5], ind->peer_addr.addr[4],
              ind->peer_addr.addr[3], ind->peer_addr.addr[2],
              ind->peer_addr.addr[1], ind->peer_addr.addr[0]);

        /* Start inactivity timer — will reboot if no DFU data arrives */
        if (g_env.inactivity_timer)
            rt_timer_start(g_env.inactivity_timer);

#ifdef PKG_USING_LITTLEVGL2RTT
        dfu_ui_update_message(UI_MSG_UPDATE_BLE, UI_MSG_DATA_BLE_CONNECTED);
        dfu_ui_update_message(UI_MSG_UPDATE_SOURCE, UI_MSG_DATA_SOURCE_BLE);
#endif
        break;
    }

    case BLE_GAP_UPDATE_CONN_PARAM_IND:
    {
        ble_gap_update_conn_param_ind_t *ind =
            (ble_gap_update_conn_param_ind_t *)data;
        LOG_I("conn interval updated: %d", ind->con_interval);
        break;
    }

    case SIBLES_MTU_EXCHANGE_IND:
    {
        sibles_mtu_exchange_ind_t *ind = (sibles_mtu_exchange_ind_t *)data;
        g_env.mtu = ind->mtu;
        LOG_I("MTU exchanged: %d", ind->mtu);
        break;
    }

    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        LOG_I("BLE disconnected: reason=%d", ind->reason);
#ifdef PKG_USING_LITTLEVGL2RTT
        dfu_ui_update_message(UI_MSG_UPDATE_BLE, UI_MSG_DATA_BLE_DISCONNECTED);
#endif
        break;
    }

    default:
        break;
    }
    return 0;
}

/* Static registration — BLE stack calls this automatically */
BLE_EVENT_REGISTER(ble_dfu_event_handler, NULL);

/*============================================================================
 * main
 *============================================================================*/

int main(void)
{
    LOG_I("=== DFU-BLE V2 Subprogram Start ===");
    LOG_I("HCPU_FLASH=0x%08X SIZE=0x%08X, OTA55X=%d",
        HCPU_FLASH_CODE_START_ADDR, HCPU_FLASH_CODE_SIZE,
    #ifdef OTA_55X
        1
    #else
        0
    #endif
        );
    /*--- 1. Init DFU V2 middleware (Engine + BLE Transport + Host Mode) ---*/
    dfu_config_t cfg = {
        .callback  = on_dfu_event,
        .user_data = RT_NULL,
    };

    if (dfu_init(&cfg) != 0)
    {
        LOG_E("dfu_init failed");
        /* Continue anyway to allow BLE diagnostics */
    }
    else
    {
        LOG_I("DFU V2 middleware initialized");

        /*
         * DFU subprogram runs from DFU/OTA_MANAGER partition (img_id=6).
         * Tell Host Mode to skip DFU images — we can't overwrite ourselves.
         * HCPU and other images will be written normally.
         */
        dfu_mode_host_set_self_img_id(6);   /* 6 = OTA_MANAGER/DFU */
    }

    /* Check for stale install flags from interrupted sessions */
    dfu_check_install();

    /*
     * Use V2's mailbox as the shared mailbox for both BLE events
     * and DFU messages. This is the key: BLE_POWER_ON_IND goes to
     * the same mailbox as DFU_MSG_BLE_DATA.
     */
    g_env.mb = dfu_get_mailbox();
    if (!g_env.mb)
    {
        /* Fallback — create our own */
        static struct rt_mailbox s_mb;
        static rt_ubase_t s_mb_pool[32];
        rt_mb_init(&s_mb, "dfu_mb", s_mb_pool,
                   sizeof(s_mb_pool) / sizeof(s_mb_pool[0]),
                   RT_IPC_FLAG_FIFO);
        g_env.mb = &s_mb;
    }

    /*--- 2. UI (optional) ---*/
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
        dfu_ui_update_message(UI_MSG_UPDATE_BLE, UI_MSG_DATA_BLE_DISCONNECTED);
        dfu_ui_update_message(UI_MSG_UPDATE_PROGRESS, "0");
    }
#endif

    /*--- 3. Enable BLE stack ---*/
    sifli_ble_enable();
    LOG_I("BLE stack enabling...");

    /*--- 4. Start timeout timers ---*/
    g_env.global_timer = rt_timer_create(
        "dfu_gto",
        dfu_global_timeout_handler, RT_NULL,
        rt_tick_from_millisecond(DFU_MODE_TIMEOUT_MS),
        RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    if (g_env.global_timer)
        rt_timer_start(g_env.global_timer);

    g_env.inactivity_timer = rt_timer_create(
        "dfu_ito",
        dfu_inactivity_timeout_handler, RT_NULL,
        rt_tick_from_millisecond(DFU_INACTIVITY_TIMEOUT_MS),
        RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    /* Inactivity timer starts when BLE connects — not here */

    /*--- 5. Main message loop ---*/
    rt_ubase_t msg;

    while (1)
    {
        if (rt_mb_recv(g_env.mb, &msg, RT_WAITING_FOREVER) != RT_EOK)
            continue;

        /* Let V2 middleware handle DFU messages first */
        if (dfu_process(msg) == 0)
            continue;

        /* V2 didn't recognize it — handle BLE stack events */
        switch (msg)
        {
        case BLE_POWER_ON_IND:
            LOG_I("BLE power on, initializing services");
            g_env.is_power_on = 1;
            g_env.mtu = 23;  /* default */

#ifdef BSP_BLE_SERIAL_TRANSMISSION
            /* Init serial transmission service (required for DFU data channel) */
            ble_serial_tran_init();
#endif
            /* Start advertising so mobile APP can find and connect to us */
            dfu_ble_advertising_start();
            LOG_I("BLE DFU ready, waiting for connection...");
            break;

        default:
            LOG_D("unhandled msg: %lu", (unsigned long)msg);
            break;
        }
    }

    return RT_EOK;
}

/*============================================================================
 * Enable DIS (Device Information Service) for compatibility
 *============================================================================*/

uint8_t ble_app_dis_enable(void)
{
    return 1;
}

/*============================================================================
 * Finsh Debug Commands
 *============================================================================*/

#ifdef RT_USING_FINSH
#include <finsh.h>

static void dfu_ble_info(int argc, char **argv)
{
    rt_kprintf("\n=== DFU BLE V2 Info ===\n");
    rt_kprintf("BLE power on: %s\n", g_env.is_power_on ? "Yes" : "No");
    rt_kprintf("MTU: %u\n", g_env.mtu);

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
MSH_CMD_EXPORT(dfu_ble_info, Show DFU BLE V2 information);

#endif