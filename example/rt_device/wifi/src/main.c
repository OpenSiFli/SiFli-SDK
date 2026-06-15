/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "rtthread.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bf0_hal.h"
#include "drv_io.h"
#include "drv_flash.h"
#ifdef RT_USING_DFS_ELMFAT
    #include "dfs_file.h"
#endif
#include "drv_flash.h"
#include <wlan_mgnt.h>
#include <wlan_cfg.h>
#include <wlan_prot.h>

#ifndef FS_REGION_START_ADDR
    #error "Need to define file system start address!"
#else
    #define FS_ROOT "root"
    #define FS_ROOT_PATH "/"
#endif

/* ============================================================
 *  User Configuration - Modify the macros below to set WiFi
 * ============================================================ */
#define WIFI_SSID       "wifi_ssid"     /* WiFi SSID to connect */
#define WIFI_PASSWORD   "wifi_password"         /* WiFi password, set to RT_NULL if open */

/* Delay(ms) after boot to wait for WiFi driver initialization */
#define WIFI_READY_WAIT_MS      5000
/* Delay(ms) between scan complete and connect */
#define WIFI_SCAN_TO_JOIN_MS    1000


#ifdef RT_USING_DFS_ELMFAT
int mnt_init(void)
{
    rt_kprintf("0x%x %d\n", FS_REGION_START_ADDR, FS_REGION_SIZE);
    register_mtd_device(FS_REGION_START_ADDR, 0x200000, FS_ROOT);
    if (dfs_mount(FS_ROOT, FS_ROOT_PATH, "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", FS_ROOT) == 0)
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(FS_ROOT, "/", "elm", 0, 0) == 0)
                rt_kprintf("mount fs on flash success\n");
            else
            {
                rt_kprintf("mount to fs on flash fail\n");
                return RT_ERROR;
            }
        }
        else
        {
            rt_kprintf("dfs_mkfs elm flash fail\n");
            return RT_ERROR;
        }
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);
#endif


/**
 * @brief  Convert security type to string description
 */
static const char *security_to_str(rt_wlan_security_t security)
{
    switch (security)
    {
    case SECURITY_OPEN:
        return "OPEN";
    case SECURITY_WEP_PSK:
        return "WEP_PSK";
    case SECURITY_WEP_SHARED:
        return "WEP_SHARED";
    case SECURITY_WPA_TKIP_PSK:
        return "WPA_TKIP_PSK";
    case SECURITY_WPA_AES_PSK:
        return "WPA_AES_PSK";
    case SECURITY_WPA2_AES_PSK:
        return "WPA2_AES_PSK";
    case SECURITY_WPA2_TKIP_PSK:
        return "WPA2_TKIP_PSK";
    case SECURITY_WPA2_MIXED_PSK:
        return "WPA2_MIXED_PSK";
    case SECURITY_WPS_OPEN:
        return "WPS_OPEN";
    case SECURITY_WPS_SECURE:
        return "WPS_SECURE";
    default:
        return "UNKNOWN";
    }
}

/**
 * @brief  Perform WiFi scan and print results
 * @retval Number of APs found, 0 on failure
 */
static int wifi_do_scan(void)
{
    struct rt_wlan_scan_result *scan_result = RT_NULL;
    int num = 0;

    rt_kprintf("\n[WiFi] Start scanning...\n");
    scan_result = rt_wlan_scan_sync();
    if (scan_result)
    {
        int index;
        num = scan_result->num;

        rt_kprintf("[WiFi] Scan done, found %d APs:\n\n", num);
        rt_kprintf("             SSID                      MAC            security    rssi chn Mbps\n");
        rt_kprintf("------------------------------- -----------------  -------------- ---- --- ----\n");

        for (index = 0; index < num; index++)
        {
            rt_kprintf("%-32.32s", &scan_result->info[index].ssid.val[0]);
            rt_kprintf("%02x:%02x:%02x:%02x:%02x:%02x  ",
                       scan_result->info[index].bssid[0],
                       scan_result->info[index].bssid[1],
                       scan_result->info[index].bssid[2],
                       scan_result->info[index].bssid[3],
                       scan_result->info[index].bssid[4],
                       scan_result->info[index].bssid[5]);
            rt_kprintf("%-14.14s ", security_to_str(scan_result->info[index].security));
            rt_kprintf("%-4d ", scan_result->info[index].rssi);
            rt_kprintf("%3d ", scan_result->info[index].channel);
            rt_kprintf("%4d\n", scan_result->info[index].datarate / 1000000);
        }

        rt_wlan_scan_result_clean();
    }
    else
    {
        rt_kprintf("[WiFi] Scan failed, no results\n");
    }

    return num;
}

/**
 * @brief  Connect to a specified WiFi AP
 * @param  ssid      WiFi SSID
 * @param  password  WiFi password (RT_NULL if open network)
 * @retval RT_EOK on success
 */
static rt_err_t wifi_do_connect(const char *ssid, const char *password)
{
    rt_err_t ret;

    rt_kprintf("\n[WiFi] Connecting to: %s ...\n", ssid);

    ret = rt_wlan_connect(ssid, password);
    if (ret != RT_EOK)
    {
        rt_kprintf("[WiFi] Connect failed! error: %d\n", ret);
    }
    else
    {
        rt_kprintf("[WiFi] Connect request sent, waiting for authentication...\n");
    }

    return ret;
}

/**
 * @brief  WiFi auto scan and connect thread entry
 */
static void wifi_auto_connect_thread(void *parameter)
{
    int scan_num;

    /* Wait for WiFi driver initialization */
    rt_kprintf("\n[WiFi] Waiting for driver init (%d ms)...\n", WIFI_READY_WAIT_MS);
    rt_thread_mdelay(WIFI_READY_WAIT_MS);

    /* Step 1: Scan */
    scan_num = wifi_do_scan();
    if (scan_num == 0)
    {
        rt_kprintf("[WiFi] No AP found, retry in 3 seconds...\n");
        rt_thread_mdelay(3000);
        scan_num = wifi_do_scan();
    }

    /* Wait before connecting */
    rt_thread_mdelay(WIFI_SCAN_TO_JOIN_MS);

    /* Step 2: Connect */
    rt_kprintf("\n[WiFi] Target AP: SSID=\"%s\"\n", WIFI_SSID);
    wifi_do_connect(WIFI_SSID, WIFI_PASSWORD);
}


/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_thread_t tid;

    rt_kprintf("wifi start\n");

    /* Create WiFi auto-connect thread to avoid blocking main */
    tid = rt_thread_create("wifi_auto",
                           wifi_auto_connect_thread,
                           RT_NULL,
                           4096,
                           20,
                           10);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        rt_kprintf("[WiFi] Failed to create auto-connect thread!\n");
    }

    /* Main loop */
    while (1)
    {
        rt_thread_mdelay(10000);
    }
    return 0;
}
