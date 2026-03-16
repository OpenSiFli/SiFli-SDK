/**
 * @file main.c
 * @brief PAN OTA V2 Example — User Application
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include <stdio.h>
#include <string.h>

/* Network (for DNS test command) */
#include "lwip/dns.h"

/* BT stack */
#include "bts2_app_inc.h"
#include "ble_connection_manager.h"
#include "bt_connection_manager.h"

/* DFU V2 — firmware info flash API + macro definitions */
#include "dfu_macro.h"
#include "dfu_fwinfo.h"

/* OTA network — server registration + version query */
#include "ota_network.h"

#define LOG_TAG "pan_ota"
#include <ulog.h>

/*============================================================================
 * File system mount
 *============================================================================*/

#if defined(BSP_USING_SPI_NAND) && defined(RT_USING_DFS)
#include "dfs_file.h"
#include "dfs_posix.h"
#include "drv_flash.h"
#define NAND_MTD_NAME "root"
int mnt_init(void)
{
    register_nand_device(FS_REGION_START_ADDR & (0xFC000000),
                         FS_REGION_START_ADDR - (FS_REGION_START_ADDR & (0xFC000000)),
                         FS_REGION_SIZE, NAND_MTD_NAME);
    if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", NAND_MTD_NAME) == 0)
        {
            rt_kprintf("make elm fs on flash success, mount again\n");
            if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
                rt_kprintf("mount fs on flash success\n");
            else
                rt_kprintf("mount to fs on flash fail\n");
        }
        else
            rt_kprintf("dfs_mkfs elm flash fail\n");
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);
#endif

/*============================================================================
 * Constants
 *============================================================================*/

#define APP_VERSION         "V1.0"
#define BT_APP_READY        1
#define BT_APP_CONNECT_PAN  2
#define PAN_TIMER_MS        3000

#ifdef BT_DEVICE_NAME
static const char *local_name = BT_DEVICE_NAME;
#else
static const char *local_name = "sifli_pan";
#endif

/*============================================================================
 * BT App Environment
 *============================================================================*/

typedef struct
{
    BOOL                   bt_connected;
    bt_notify_device_mac_t bd_addr;
    rt_timer_t             pan_connect_timer;
    uint8_t                pan_connected;
    uint8_t                retry_flag;
    uint8_t                retry_times;
    uint8_t                retry_max_times;
} bt_app_t;

static bt_app_t     g_bt_app_env;
static rt_mailbox_t g_bt_app_mb;
static char         g_latest_version[32] = {0};

/*============================================================================
 * BT Helpers
 *============================================================================*/

void bt_pan_set_retry_flag(uint8_t enable)
{
    g_bt_app_env.retry_flag = enable;
}

void bt_pan_set_retry_times(uint8_t times)
{
    g_bt_app_env.retry_max_times = times;
}

static void bt_app_connect_pan_timeout_handle(void *parameter)
{
    (void)parameter;
    if (g_bt_app_mb && g_bt_app_env.bt_connected)
        rt_mb_send(g_bt_app_mb, BT_APP_CONNECT_PAN);
}

static void pan_reconnect(void)
{
    const int reconnect_interval_ms = 10000;
    while (g_bt_app_env.retry_times < g_bt_app_env.retry_max_times)
    {
        LOG_I("PAN reconnect attempt %d", g_bt_app_env.retry_times + 1);
        if (!g_bt_app_env.retry_flag)
            return;

        if (g_bt_app_env.pan_connect_timer)
            rt_timer_stop(g_bt_app_env.pan_connect_timer);

        bt_interface_conn_ext((char *)&g_bt_app_env.bd_addr, BT_PROFILE_HID);
        g_bt_app_env.retry_times++;
        rt_thread_mdelay(reconnect_interval_ms);

        if (g_bt_app_env.pan_connected)
        {
            LOG_I("PAN reconnected successfully");
            g_bt_app_env.retry_times = 0;
            return;
        }
    }
    g_bt_app_env.retry_times = 0;
}

/*============================================================================
 * BT Event Callback
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
            rt_mb_send(g_bt_app_mb, BT_APP_READY);
            break;

        case BT_NOTIFY_COMMON_ACL_DISCONNECTED:
        {
            bt_notify_device_base_info_t *info = (bt_notify_device_base_info_t *)data;
            LOG_I("ACL disconnected (0x%.2x:%.2x:%.2x:%.2x:%.2x:%.2x) res=%d",
                  info->mac.addr[5], info->mac.addr[4], info->mac.addr[3],
                  info->mac.addr[2], info->mac.addr[1], info->mac.addr[0],
                  info->res);
            g_bt_app_env.bt_connected = FALSE;
            if (g_bt_app_env.pan_connect_timer)
                rt_timer_stop(g_bt_app_env.pan_connect_timer);
            break;
        }

        case BT_NOTIFY_COMMON_ENCRYPTION:
        {
            bt_notify_device_mac_t *mac = (bt_notify_device_mac_t *)data;
            LOG_I("Encryption complete");
            g_bt_app_env.bd_addr = *mac;
            pan_conn = 1;
            break;
        }

        case BT_NOTIFY_COMMON_PAIR_IND:
        {
            bt_notify_device_base_info_t *info = (bt_notify_device_base_info_t *)data;
            LOG_I("Pairing complete: %d", info->res);
            if (info->res == BTS2_SUCC)
            {
                g_bt_app_env.bd_addr = info->mac;
                pan_conn = 1;
            }
            break;
        }

        case BT_NOTIFY_COMMON_KEY_MISSING:
        {
            bt_notify_device_base_info_t *info = (bt_notify_device_base_info_t *)data;
            LOG_I("Key missing %d", info->res);
            memset(&g_bt_app_env.bd_addr, 0xFF, sizeof(g_bt_app_env.bd_addr));
            bt_cm_delete_bonded_devs_and_linkkey(info->mac.addr);
            break;
        }

        default:
            break;
        }

        if (pan_conn)
        {
            LOG_I("bd_addr: 0x%.2x:%.2x:%.2x:%.2x:%.2x:%.2x",
                  g_bt_app_env.bd_addr.addr[5], g_bt_app_env.bd_addr.addr[4],
                  g_bt_app_env.bd_addr.addr[3], g_bt_app_env.bd_addr.addr[2],
                  g_bt_app_env.bd_addr.addr[1], g_bt_app_env.bd_addr.addr[0]);
            g_bt_app_env.bt_connected = TRUE;

            if (!g_bt_app_env.pan_connect_timer)
            {
                g_bt_app_env.pan_connect_timer = rt_timer_create(
                    "conn_pan", bt_app_connect_pan_timeout_handle, NULL,
                    rt_tick_from_millisecond(PAN_TIMER_MS),
                    RT_TIMER_FLAG_SOFT_TIMER);
            }
            else
            {
                rt_timer_stop(g_bt_app_env.pan_connect_timer);
            }
            rt_timer_start(g_bt_app_env.pan_connect_timer);
        }
    }
    else if (type == BT_NOTIFY_PAN)
    {
        switch (event_id)
        {
        case BT_NOTIFY_PAN_PROFILE_CONNECTED:
            LOG_I("PAN connected");
            if (g_bt_app_env.pan_connect_timer)
                rt_timer_stop(g_bt_app_env.pan_connect_timer);
            g_bt_app_env.pan_connected = 1;
            break;

        case BT_NOTIFY_PAN_PROFILE_DISCONNECTED:
            LOG_I("PAN disconnected");
            g_bt_app_env.pan_connected = 0;
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
            if (!g_bt_app_env.pan_connected)
            {
                if (g_bt_app_env.pan_connect_timer)
                    rt_timer_stop(g_bt_app_env.pan_connect_timer);
                bt_interface_conn_ext((char *)&g_bt_app_env.bd_addr,
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

uint32_t bt_get_class_of_device(void)
{
    return (uint32_t)BT_SRVCLS_NETWORK | BT_DEVCLS_LAP | BT_LAP_FULLY;
}

/*============================================================================
 * main
 *============================================================================*/

int main(void)
{
    LOG_I("=== PAN OTA V2 User APP ===");
    LOG_I("Version: %s", APP_VERSION);

    g_bt_app_mb = rt_mb_create("bt_app", 8, RT_IPC_FLAG_FIFO);

#ifdef BSP_BT_CONNECTION_MANAGER
    bt_cm_set_profile_target(BT_CM_HID, BT_LINK_PHONE, 1);
#endif

    bt_interface_register_bt_event_notify_callback(bt_event_handle);

    /* Auto reconnect PAN */
    bt_pan_set_retry_flag(1);
    bt_pan_set_retry_times(5);

    sifli_ble_enable();

    while (1)
    {
        uint32_t value;
        if (RT_EOK == rt_mb_recv(g_bt_app_mb, (rt_uint32_t *)&value, 8000) &&
            value == BT_APP_READY)
        {
            LOG_I("BT/BLE stack ready");
        }
        else
        {
            LOG_I("BT/BLE stack init timeout");
        }

        /* Set BT device name */
        bt_interface_set_local_name(strlen(local_name), (void *)local_name);

        /* Wait for PAN connection */
        rt_mb_recv(g_bt_app_mb, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BT_APP_CONNECT_PAN)
        {
            if (g_bt_app_env.bt_connected)
                bt_interface_conn_ext((char *)&g_bt_app_env.bd_addr,
                                     BT_PROFILE_PAN);
        }
    }

    return 0;
}

/*============================================================================
 * Finsh Commands
 *============================================================================*/

#ifdef RT_USING_FINSH
#include <finsh.h>

/**
 * @brief Check for new firmware version on OTA server
 *
 * Flow: register device → query version → if newer: write fwinfo to flash
 * After this, run "ota_go" to set update flags and reboot.
 */
static void ota_check(int argc, char **argv)
{
    LOG_I("Checking for firmware update...");

    /* 1. Register device with server */
    int reg_result = ota_register_device();
    if (reg_result != 0)
        LOG_W("Device registration failed (continuing anyway)");

    /* 2. Build query URL with chip_id */
    char *chip_id = ota_get_chip_id();
    char *url = ota_build_query_url(chip_id);

    /* 3. Query server for latest version */
    int result = ota_query_latest_version(url, APP_VERSION,
                                          g_latest_version,
                                          sizeof(g_latest_version));

    if (result > 0)
    {
        LOG_I("New version available: %s (current: %s)",
              g_latest_version, APP_VERSION);
        LOG_I("Firmware info written to flash.");
        LOG_I("Run 'ota_go' to start update.");
    }
    else if (result == 0)
    {
        LOG_I("Current version is latest: %s", APP_VERSION);
    }
    else
    {
        LOG_E("Version query failed");
    }
}
MSH_CMD_EXPORT(ota_check, Check OTA server for new firmware version);

/**
 * @brief Set update flags and reboot to DFU subprogram
 *
 * Prerequisites: "ota_check" must have found a new version and written
 * firmware info to flash.
 */
static void ota_go(int argc, char **argv)
{
    /* Check if there are any files that need updating */
    BOOL needs_update = RT_FALSE;
    for (int i = 0; i < DFU_MAX_FW_FILES; i++)
    {
        struct dfu_fw_info info;
        if (dfu_fwinfo_get(i, &info) == 0 &&
            info.name[0] != '\0' && info.name[0] != 0xFF)
        {
            needs_update = RT_TRUE;
            break;
        }
    }

    if (!needs_update)
    {
        LOG_I("No firmware files found. Run 'ota_check' first.");
        return;
    }

    /* Set update flags (needs_update=1, magic) for all valid entries */
    if (dfu_fwinfo_set_update_flags() != 0)
    {
        LOG_E("Failed to set update flags");
        return;
    }

    LOG_I("Update flags set. Rebooting to DFU mode in 2s...");
    rt_thread_mdelay(2000);
    HAL_PMU_Reboot();
}
MSH_CMD_EXPORT(ota_go, Set update flags and reboot to DFU subprogram);

/**
 * @brief Show current firmware version
 */
static void ota_version(int argc, char **argv)
{
    LOG_I("Current firmware version: %s", APP_VERSION);
}
MSH_CMD_EXPORT(ota_version, Show current firmware version);

/**
 * @brief Print firmware info stored in flash
 */
static void ota_print(int argc, char **argv)
{
    LOG_I("=== Firmware Info ===");
    LOG_I("Base address: 0x%08X", DFU_FWINFO_BASE_ADDR);

    for (int i = 0; i < DFU_MAX_FW_FILES; i++)
    {
        struct dfu_fw_info info;
        if (dfu_fwinfo_get(i, &info) == 0 &&
            info.name[0] != '\0' && info.name[0] != 0xFF)
        {
            LOG_I("[%d] %s", i, info.name);
            rt_kprintf("  url: %s\n", info.url);
            LOG_I("  addr=0x%08X size=%u crc=0x%08X region=0x%08X",
                  info.addr, info.size, info.crc32, info.region_size);
            LOG_I("  file_id=%u needs_update=%u magic=0x%08X",
                  info.file_id, info.needs_update, info.magic);
        }
    }
}
MSH_CMD_EXPORT(ota_print, Print firmware info from flash);

/**
 * @brief Clear all firmware info from flash
 */
static void ota_clear(int argc, char **argv)
{
    dfu_fwinfo_clear();
    LOG_I("Firmware info cleared");
}
MSH_CMD_EXPORT(ota_clear, Clear all firmware info);

/**
 * @brief Miscellaneous PAN commands
 */
static void pan_cmd(int argc, char **argv)
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
        bt_app_connect_pan_timeout_handle(NULL);
    }
    else if (strcmp(argv[1], "autoconnect") == 0)
    {
        pan_reconnect();
    }
    else if (strcmp(argv[1], "set_retry_flag") == 0 && argc > 2)
    {
        bt_pan_set_retry_flag(atoi(argv[2]));
    }
    else if (strcmp(argv[1], "set_retry_times") == 0 && argc > 2)
    {
        bt_pan_set_retry_times(atoi(argv[2]));
    }
}
MSH_CMD_EXPORT(pan_cmd, PAN commands: del_bond|conn_pan|autoconnect|set_retry_flag|set_retry_times);

/**
 * @brief Test firmware info update flags read/write cycle
 *
 * Ported from old dfu_pan_test_update_flags(). Exercises the full
 * flag lifecycle: print initial state → set all flags to 1 → verify →
 * clear all flags to 0 → verify → clean up.
 */
static void ota_test_flags(int argc, char **argv)
{
    LOG_I("=== Update Flags Test ===");

    /* 1. Print initial state */
    LOG_I("1. Initial state:");
    for (int i = 0; i < DFU_MAX_FW_FILES; i++)
    {
        struct dfu_fw_info info;
        if (dfu_fwinfo_get(i, &info) == 0)
            LOG_I("  [%d] needs_update=%u magic=0x%08X", i, info.needs_update, info.magic);
        else
            LOG_I("  [%d] read failed", i);
    }

    /* 2. Write test entries with needs_update=1 */
    LOG_I("2. Setting all flags to 1:");
    for (int i = 0; i < DFU_MAX_FW_FILES; i++)
    {
        struct dfu_fw_info info;
        memset(&info, 0, sizeof(info));
        rt_snprintf(info.name, sizeof(info.name), "test_entry_%d", i);
        info.needs_update = 1;
        info.magic = DFU_FW_MAGIC;
        dfu_fwinfo_set(i, &info);
    }

    /* 3. Verify flags are set */
    LOG_I("3. Verifying flags = 1:");
    for (int i = 0; i < DFU_MAX_FW_FILES; i++)
    {
        struct dfu_fw_info info;
        if (dfu_fwinfo_get(i, &info) == 0)
            LOG_I("  [%d] needs_update=%u %s", i, info.needs_update,
                  (info.needs_update == 1) ? "OK" : "FAIL");
        else
            LOG_I("  [%d] read failed", i);
    }

    /* 4. Clear all */
    LOG_I("4. Clearing all flags:");
    dfu_fwinfo_clear();

    /* 5. Verify cleared */
    LOG_I("5. Verifying flags cleared:");
    for (int i = 0; i < DFU_MAX_FW_FILES; i++)
    {
        struct dfu_fw_info info;
        if (dfu_fwinfo_get(i, &info) == 0)
            LOG_I("  [%d] needs_update=%u name='%s' %s", i, info.needs_update,
                  info.name,
                  (info.needs_update == 0 && info.name[0] == '\0') ? "OK" : "FAIL");
        else
            LOG_I("  [%d] read failed", i);
    }

    LOG_I("=== Test Complete ===");
}
MSH_CMD_EXPORT(ota_test_flags, Test firmware info update flags read/write cycle);

/**
 * @brief Test DNS resolution for OTA server
 *
 * Ported from old test_dns_resolution_cmd().
 */
static void ota_test_dns(int argc, char **argv)
{
    const char *hostname = "ota.sifli.com";
    if (argc > 1)
        hostname = argv[1];

    LOG_I("Testing DNS for: %s", hostname);

    ip_addr_t addr = {0};
    err_t err = dns_gethostbyname(hostname, &addr, NULL, NULL);

    if (err == ERR_OK || err == ERR_INPROGRESS)
        LOG_I("DNS OK for %s", hostname);
    else
        LOG_E("DNS FAILED for %s, err=%d", hostname, err);
}
MSH_CMD_EXPORT(ota_test_dns, Test DNS resolution [hostname]);

#endif /* RT_USING_FINSH */
