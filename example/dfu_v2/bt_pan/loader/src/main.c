/**
 * @file main.c
 * @brief DFU-PAN V2
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdio.h>
#include <string.h>

/* BT stack */
#include "bts2_app_inc.h"
#include "bt_connection_manager.h"
#include "ble_connection_manager.h"
#include "bf0_sibles_internal.h"

/* Network */
#include <lwip/sys.h>
#include "lwip/tcpip.h"
#include "lwip/dns.h"
#include <netdb.h>

/* DFU V2 middleware */
#include "dfu_v2.h"
#include "dfu_macro.h"
#include "drv_flash.h"

/* UI */
#ifdef PKG_USING_LITTLEVGL2RTT
#include "dfu_ui.h"
#endif

#define LOG_TAG "dfu.pan"
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

#define BT_APP_READY                    1
#define BT_APP_CONNECT_PAN              2
#define OTA_BT_APP_CONNECT_PAN_SUCCESS  3
#define OTA_AUTO_UPDATE_CMD             4

#define PAN_TIMER_MS        3000
#define BLUETOOTH_NAME      "sifli-pan"

#define UI_THREAD_STACK     8192
#define UI_THREAD_PRIO      30

/*============================================================================
 * BT app environment
 *============================================================================*/

typedef struct
{
    BOOL bt_connected;
    bt_notify_device_mac_t bd_addr;
    rt_timer_t pan_connect_timer;
} bt_app_env_t;

static bt_app_env_t g_bt_env;
static rt_mailbox_t g_bt_app_mb;
static BOOL g_pan_connected = FALSE;
static rt_timer_t g_auto_update_timer = RT_NULL;

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
        dfu_ui_update_message(UI_MSG_UPDATE_PROGRESS_COLOR,
                              PROGRESS_COLOR_SUCCESS);
        dfu_ui_update_message(UI_MSG_SHOW_SUCCESS_POPUP, RT_NULL);
#endif
        break;

    case DFU_EVT_ERROR:
        LOG_E("DFU error: %d", param->err.error);
#ifdef PKG_USING_LITTLEVGL2RTT
        dfu_ui_update_message(UI_MSG_UPDATE_PROGRESS_COLOR,
                              PROGRESS_COLOR_ERROR);
        dfu_ui_update_message(UI_MSG_SHOW_FAILURE_POPUP, "更新失败");
#endif
        break;

    default:
        break;
    }
    return 0;
}

/*============================================================================
 * BT Helpers
 *============================================================================*/

static void exit_sniff_mode(void)
{
    LOG_I("exit sniff mode");
    bt_interface_exit_sniff_mode(
        (unsigned char *)&g_bt_env.bd_addr);
    bt_interface_wr_link_policy_setting(
        (unsigned char *)&g_bt_env.bd_addr,
        BT_NOTIFY_LINK_POLICY_ROLE_SWITCH);
}

static void pan_connect_timeout_handle(void *parameter)
{
    (void)parameter;
    LOG_I("pan connect timer fired, bt_connected=%d", g_bt_env.bt_connected);
    if (g_bt_app_mb && g_bt_env.bt_connected)
        rt_mb_send(g_bt_app_mb, BT_APP_CONNECT_PAN);
}

static void auto_update_check_callback(void *parameter)
{
    (void)parameter;
    LOG_I("auto update timer: pan_connected=%d", g_pan_connected);
    if (g_pan_connected)
    {
        rt_thread_mdelay(1000);  /* stability delay */
        rt_mb_send(g_bt_app_mb, OTA_AUTO_UPDATE_CMD);
    }
}

/*============================================================================
 * DNS check with retry
 *============================================================================*/

static int dns_check(const char *hostname, int max_retries)
{
    for (int i = 0; i < max_retries; i++)
    {
        ip_addr_t addr = {0};
        err_t err = dns_gethostbyname(hostname, &addr, NULL, NULL);
        if (err == ERR_OK || err == ERR_INPROGRESS)
        {
            LOG_I("DNS OK for %s (attempt %d)", hostname, i + 1);
            return 0;
        }
        LOG_W("DNS fail for %s, attempt %d/%d", hostname, i + 1, max_retries);
        if (i < max_retries - 1)
            rt_thread_mdelay(1000);
    }
    return -1;
}

/*============================================================================
 * Read firmware info from flash → build dfu_download_req
 *============================================================================*/

static int read_firmware_info(struct dfu_fw_info *out, int max_count)
{
    int valid = 0;
    for (int i = 0; i < max_count; i++)
    {
        uint32_t addr = DFU_FWINFO_BASE_ADDR +
                        i * sizeof(struct dfu_fw_info);
        int n = rt_flash_read(addr, (uint8_t *)&out[valid],
                              sizeof(struct dfu_fw_info));
        if (n != sizeof(struct dfu_fw_info))
            continue;

        /* Valid entry: non-empty name */
        if (out[valid].name[0] != '\0' && out[valid].name[0] != 0xFF)
        {
            LOG_I("fw[%d]: %s addr=0x%08x size=%u",
                  valid, out[valid].name, out[valid].addr, out[valid].size);
            valid++;
        }
    }
    return valid;
}

static void clear_firmware_info(void)
{
    int data_size = sizeof(struct dfu_fw_info) * DFU_MAX_FW_FILES;
    int align = rt_flash_get_erase_alignment(DFU_FWINFO_BASE_ADDR);
    int erase_size = ((data_size + align - 1) / align) * align;

    struct dfu_fw_info empty[DFU_MAX_FW_FILES];
    memset(empty, 0, sizeof(empty));

    rt_flash_erase(DFU_FWINFO_BASE_ADDR, erase_size);
    rt_flash_write(DFU_FWINFO_BASE_ADDR, (uint8_t *)empty, data_size);
    LOG_I("firmware info cleared");
}

/*============================================================================
 * Execute OTA: read flash → V2 download → clear → reboot
 *============================================================================*/

static void execute_ota_update(void)
{
    /* 1. DNS check */
    if (dns_check("ota.sifli.com", 10) != 0)
    {
        LOG_E("DNS failed, aborting OTA");
#ifdef PKG_USING_LITTLEVGL2RTT
        dfu_ui_update_message(UI_MSG_SHOW_FAILURE_POPUP, "连接网络后重试");
#endif
        return;
    }

    /* 2. Exit sniff mode for stable data transfer */
    exit_sniff_mode();

    /* 3. Read firmware file info from flash */
    struct dfu_fw_info fw_files[DFU_MAX_FW_FILES];
    memset(fw_files, 0, sizeof(fw_files));
    int file_count = read_firmware_info(fw_files, DFU_MAX_FW_FILES);

    if (file_count <= 0)
    {
        LOG_E("no valid firmware files in flash");
        return;
    }

    LOG_I("found %d firmware file(s), starting V2 download", file_count);

#ifdef PKG_USING_LITTLEVGL2RTT
    dfu_ui_update_message(UI_MSG_UPDATE_SOURCE, UI_MSG_DATA_SOURCE_PAN);
    dfu_ui_update_message(UI_MSG_UPDATE_PROGRESS, "0");
#endif

    /* 4. Build V2 download request from flash entries */
    dfu_file_t v2_files[DFU_MAX_FW_FILES];
    for (int i = 0; i < file_count; i++)
    {
        v2_files[i].url         = fw_files[i].url;
        v2_files[i].flash_addr  = fw_files[i].addr;
        v2_files[i].file_size   = fw_files[i].size;
        v2_files[i].file_crc    = fw_files[i].crc32;
        v2_files[i].region_size = fw_files[i].region_size;
        v2_files[i].img_id      = (uint8_t)fw_files[i].file_id;

#ifdef PKG_USING_LITTLEVGL2RTT
        dfu_ui_update_message(UI_MSG_UPDATE_FILES, fw_files[i].name);
#endif
    }

    dfu_download_req_t req = {
        .files      = v2_files,
        .file_count = file_count,
    };

    /* 5. Execute blocking download via V2 middleware */
    int ret = dfu_download(&req);

    if (ret == 0)
    {
        LOG_I("download complete, clearing flash info");
        clear_firmware_info();

        LOG_I("rebooting to user APP...");
        rt_thread_mdelay(2000);  /* let UI show success */
        HAL_PMU_Reboot();
    }
    else
    {
        LOG_E("download failed: %d", ret);
#ifdef PKG_USING_LITTLEVGL2RTT
        dfu_ui_update_message(UI_MSG_SHOW_FAILURE_POPUP, "下载失败");
#endif
    }
}

/*============================================================================
 * BT event callback
 *============================================================================*/

static int bt_event_handle(uint16_t type, uint16_t event_id,
                           uint8_t *data, uint16_t data_len)
{
    if (type == BT_NOTIFY_COMMON)
    {
        int pan_conn = 0;

        switch (event_id)
        {
        case BT_NOTIFY_COMMON_BT_STACK_READY:
            LOG_I("BT stack ready");
            rt_mb_send(g_bt_app_mb, BT_APP_READY);
            break;

        case BT_NOTIFY_COMMON_ACL_CONNECTED:
#ifdef PKG_USING_LITTLEVGL2RTT
            dfu_ui_update_message(UI_MSG_UPDATE_BLE,
                                  UI_MSG_DATA_BLE_CONNECTED);
#endif
            LOG_I("ACL connected");
            break;

        case BT_NOTIFY_COMMON_ACL_DISCONNECTED:
        {
#ifdef PKG_USING_LITTLEVGL2RTT
            dfu_ui_update_message(UI_MSG_UPDATE_BLE,
                                  UI_MSG_DATA_BLE_DISCONNECTED);
#endif
            bt_notify_device_base_info_t *info =
                (bt_notify_device_base_info_t *)data;
            LOG_I("ACL disconnected (0x%.2x:%.2x:%.2x:%.2x:%.2x:%.2x) res=%d",
                  info->mac.addr[5], info->mac.addr[4], info->mac.addr[3],
                  info->mac.addr[2], info->mac.addr[1], info->mac.addr[0],
                  info->res);
            g_bt_env.bt_connected = FALSE;
            if (g_bt_env.pan_connect_timer)
                rt_timer_stop(g_bt_env.pan_connect_timer);
            break;
        }

        case BT_NOTIFY_COMMON_ENCRYPTION:
        {
            bt_notify_device_mac_t *mac = (bt_notify_device_mac_t *)data;
            LOG_I("Encryption complete");
            g_bt_env.bd_addr = *mac;
            pan_conn = 1;
            break;
        }

        case BT_NOTIFY_COMMON_PAIR_IND:
        {
            bt_notify_device_base_info_t *info =
                (bt_notify_device_base_info_t *)data;
            LOG_I("Pairing complete: %d", info->res);
            if (info->res == BTS2_SUCC)
            {
                g_bt_env.bd_addr = info->mac;
                pan_conn = 1;
            }
            break;
        }

        case BT_NOTIFY_COMMON_KEY_MISSING:
        {
            bt_notify_device_base_info_t *info =
                (bt_notify_device_base_info_t *)data;
            LOG_I("Key missing %d", info->res);
            memset(&g_bt_env.bd_addr, 0xFF, sizeof(g_bt_env.bd_addr));
            bt_cm_delete_bonded_devs_and_linkkey(info->mac.addr);
            break;
        }

        default:
            break;
        }

        if (pan_conn)
        {
            LOG_I("bd_addr: 0x%.2x:%.2x:%.2x:%.2x:%.2x:%.2x",
                  g_bt_env.bd_addr.addr[5], g_bt_env.bd_addr.addr[4],
                  g_bt_env.bd_addr.addr[3], g_bt_env.bd_addr.addr[2],
                  g_bt_env.bd_addr.addr[1], g_bt_env.bd_addr.addr[0]);
            g_bt_env.bt_connected = TRUE;

            if (!g_bt_env.pan_connect_timer)
            {
                g_bt_env.pan_connect_timer = rt_timer_create(
                    "conn_pan", pan_connect_timeout_handle, NULL,
                    rt_tick_from_millisecond(PAN_TIMER_MS),
                    RT_TIMER_FLAG_SOFT_TIMER);
            }
            else
            {
                rt_timer_stop(g_bt_env.pan_connect_timer);
            }
            rt_timer_start(g_bt_env.pan_connect_timer);
        }
    }
    else if (type == BT_NOTIFY_PAN)
    {
        switch (event_id)
        {
        case BT_NOTIFY_PAN_PROFILE_CONNECTED:
            LOG_I("PAN connected");
            if (g_bt_env.pan_connect_timer)
                rt_timer_stop(g_bt_env.pan_connect_timer);
            g_pan_connected = TRUE;
#ifdef PKG_USING_LITTLEVGL2RTT
            dfu_ui_update_message(UI_MSG_UPDATE_NET,
                                  UI_MSG_DATA_NET_CONNECTED);
#endif
            rt_mb_send(g_bt_app_mb, OTA_BT_APP_CONNECT_PAN_SUCCESS);
            break;

        case BT_NOTIFY_PAN_PROFILE_DISCONNECTED:
            LOG_I("PAN disconnected");
            g_pan_connected = FALSE;
#ifdef PKG_USING_LITTLEVGL2RTT
            dfu_ui_update_message(UI_MSG_UPDATE_NET,
                                  UI_MSG_DATA_NET_DISCONNECTED);
#endif
            break;

        default:
            break;
        }
    }
    else if (type == BT_NOTIFY_HID)
    {
        switch (event_id)
        {
        case BT_NOTIFY_HID_PROFILE_CONNECTED:
            LOG_I("HID connected");
            if (!g_pan_connected)
            {
                if (g_bt_env.pan_connect_timer)
                    rt_timer_stop(g_bt_env.pan_connect_timer);
                bt_interface_conn_ext((char *)&g_bt_env.bd_addr,
                                     BT_PROFILE_PAN);
            }
            break;

        case BT_NOTIFY_HID_PROFILE_DISCONNECTED:
            LOG_I("HID disconnected");
            break;

        default:
            break;
        }
    }

    return 0;
}

/*============================================================================
 * main
 *============================================================================*/

int main(void)
{
    LOG_I("=== DFU-PAN V2 Subprogram Start ===");

    /*--- 1. UI ---*/
#ifdef PKG_USING_LITTLEVGL2RTT
    {
        rt_err_t ret = rt_thread_init(&s_ui_thread, "dfu_ui",
                                      ui_thread_entry, RT_NULL,
                                      s_ui_stack, sizeof(s_ui_stack),
                                      UI_THREAD_PRIO, 10);
        if (ret == RT_EOK)
        {
            rt_thread_startup(&s_ui_thread);
            LOG_I("UI thread started");
        }
    }
#endif

    /*--- 2. Init DFU V2 middleware ---*/
    dfu_config_t cfg = {
        .callback  = on_dfu_event,
        .user_data = RT_NULL,
    };

    if (dfu_init(&cfg) != 0)
    {
        LOG_E("dfu_init failed");
        /* Continue anyway — BT will still connect, but download will fail */
    }
    else
    {
        LOG_I("DFU V2 middleware initialized");
    }

    /*--- 3. BT stack ---*/
    g_bt_app_mb = rt_mb_create("bt_app", 8, RT_IPC_FLAG_FIFO);

#ifdef BSP_BT_CONNECTION_MANAGER
    bt_cm_set_profile_target(BT_CM_HID, BT_LINK_PHONE, 1);
#endif

    bt_interface_register_bt_event_notify_callback(bt_event_handle);
    sifli_ble_enable();
    LOG_I("BT/BLE enabled");

    /*--- 4. Main message loop ---*/
    while (1)
    {
        uint32_t value;
        rt_mb_recv(g_bt_app_mb, (rt_uint32_t *)&value, RT_WAITING_FOREVER);

        switch (value)
        {
        case BT_APP_READY:
        {
            LOG_I("BT/BLE stack ready");

#ifdef BT_NAME_MAC_ENABLE
            char local_name[48];
            bd_addr_t addr;
            ble_get_public_address(&addr);
            rt_snprintf(local_name, sizeof(local_name),
                        "%s-%02x:%02x:%02x:%02x:%02x:%02x",
                        BLUETOOTH_NAME,
                        addr.addr[0], addr.addr[1], addr.addr[2],
                        addr.addr[3], addr.addr[4], addr.addr[5]);
            bt_interface_set_local_name(strlen(local_name), (void *)local_name);
#else
            bt_interface_set_local_name(strlen(BLUETOOTH_NAME),
                                       (void *)BLUETOOTH_NAME);
#endif
            LOG_I("BT name set");
            break;
        }

        case BT_APP_CONNECT_PAN:
            LOG_I("connecting PAN...");
            if (g_bt_env.bt_connected)
            {
                bt_interface_conn_ext((char *)&g_bt_env.bd_addr,
                                     BT_PROFILE_PAN);
            }
            break;

        case OTA_BT_APP_CONNECT_PAN_SUCCESS:
            LOG_I("PAN success, scheduling auto update check");
            if (!g_auto_update_timer)
            {
                g_auto_update_timer = rt_timer_create(
                    "auto_upd", auto_update_check_callback, RT_NULL,
                    rt_tick_from_millisecond(1000),
                    RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
            }
            else
            {
                rt_timer_stop(g_auto_update_timer);
            }
            rt_timer_start(g_auto_update_timer);
            break;

        case OTA_AUTO_UPDATE_CMD:
            LOG_I("=== Auto Update Start ===");
            execute_ota_update();
            break;

        default:
            LOG_D("unknown msg: %lu", (unsigned long)value);
            break;
        }
    }

    return RT_EOK;
}

/*============================================================================
 * Finsh Debug Commands
 *============================================================================*/

#ifdef RT_USING_FINSH
#include <finsh.h>

static void ota_cmd(int argc, char **argv)
{
    if (argc < 2) return;

    if (strcmp(argv[1], "del_bond") == 0)
    {
#ifdef BSP_BT_CONNECTION_MANAGER
        bt_cm_delete_bonded_devs();
        LOG_I("Bonds deleted");
#endif
    }
    else if (strcmp(argv[1], "conn_pan") == 0)
    {
        pan_connect_timeout_handle(NULL);
    }
    else if (strcmp(argv[1], "download") == 0)
    {
        execute_ota_update();
    }
    else if (strcmp(argv[1], "print") == 0)
    {
        struct dfu_fw_info fw[DFU_MAX_FW_FILES];
        memset(fw, 0, sizeof(fw));
        int n = read_firmware_info(fw, DFU_MAX_FW_FILES);
        LOG_I("=== %d firmware file(s) ===", n);
        for (int i = 0; i < n; i++)
        {
            LOG_I("[%d] %s", i, fw[i].name);
            rt_kprintf("  url: %s\n", fw[i].url);
            LOG_I("  addr=0x%08x size=%u crc=0x%08x region=0x%08x",
                  fw[i].addr, fw[i].size, fw[i].crc32, fw[i].region_size);
            LOG_I("  file_id=%u needs_update=%u magic=0x%08x",
                  fw[i].file_id, fw[i].needs_update, fw[i].magic);
        }
    }
    else if (strcmp(argv[1], "clear") == 0)
    {
        clear_firmware_info();
    }
    else if (strcmp(argv[1], "test_flags") == 0)
    {
        LOG_I("=== Update Flags Test ===");

        /* 1. Print initial state */
        LOG_I("1. Initial state:");
        for (int i = 0; i < DFU_MAX_FW_FILES; i++)
        {
            struct dfu_fw_info info;
            uint32_t addr = DFU_FWINFO_BASE_ADDR +
                            i * sizeof(struct dfu_fw_info);
            if (rt_flash_read(addr, (uint8_t *)&info, sizeof(info)) == sizeof(info))
                LOG_I("  [%d] needs_update=%u magic=0x%08X", i, info.needs_update, info.magic);
            else
                LOG_I("  [%d] read failed", i);
        }

        /* 2. Write test entries with needs_update=1 */
        LOG_I("2. Setting all flags to 1:");
        struct dfu_fw_info test_files[DFU_MAX_FW_FILES];
        memset(test_files, 0, sizeof(test_files));
        for (int i = 0; i < DFU_MAX_FW_FILES; i++)
        {
            rt_snprintf(test_files[i].name, sizeof(test_files[i].name), "test_%d", i);
            test_files[i].needs_update = 1;
            test_files[i].magic = DFU_FW_MAGIC;
        }
        int data_size = sizeof(struct dfu_fw_info) * DFU_MAX_FW_FILES;
        int align = rt_flash_get_erase_alignment(DFU_FWINFO_BASE_ADDR);
        int erase_size = ((data_size + align - 1) / align) * align;
        rt_flash_erase(DFU_FWINFO_BASE_ADDR, erase_size);
        rt_flash_write(DFU_FWINFO_BASE_ADDR, (uint8_t *)test_files, data_size);

        /* 3. Verify */
        LOG_I("3. Verifying flags = 1:");
        for (int i = 0; i < DFU_MAX_FW_FILES; i++)
        {
            struct dfu_fw_info info;
            uint32_t addr = DFU_FWINFO_BASE_ADDR +
                            i * sizeof(struct dfu_fw_info);
            if (rt_flash_read(addr, (uint8_t *)&info, sizeof(info)) == sizeof(info))
                LOG_I("  [%d] needs_update=%u %s", i, info.needs_update,
                      (info.needs_update == 1) ? "OK" : "FAIL");
        }

        /* 4. Clear */
        LOG_I("4. Clearing:");
        clear_firmware_info();

        /* 5. Verify cleared */
        LOG_I("5. Verifying cleared:");
        for (int i = 0; i < DFU_MAX_FW_FILES; i++)
        {
            struct dfu_fw_info info;
            uint32_t addr = DFU_FWINFO_BASE_ADDR +
                            i * sizeof(struct dfu_fw_info);
            if (rt_flash_read(addr, (uint8_t *)&info, sizeof(info)) == sizeof(info))
                LOG_I("  [%d] needs_update=%u %s", i, info.needs_update,
                      (info.needs_update == 0) ? "OK" : "FAIL");
        }
        LOG_I("=== Test Complete ===");
    }
    else if (strcmp(argv[1], "test_dns") == 0)
    {
        const char *hostname = (argc > 2) ? argv[2] : "ota.sifli.com";
        LOG_I("Testing DNS for: %s", hostname);
        ip_addr_t addr = {0};
        err_t err = dns_gethostbyname(hostname, &addr, NULL, NULL);
        if (err == ERR_OK || err == ERR_INPROGRESS)
            LOG_I("DNS OK for %s", hostname);
        else
            LOG_E("DNS FAILED for %s, err=%d", hostname, err);
    }
}
MSH_CMD_EXPORT(ota_cmd, DFU PAN V2: del_bond|conn_pan|download|print|clear|test_flags|test_dns);

#endif
