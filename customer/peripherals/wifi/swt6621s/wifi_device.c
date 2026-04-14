/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "rtthread.h"
#include "drivers/pin.h"
#include "swt6621s_wlan_port.h"
#include "skw_boot_code.h"
#include <drivers/mmcsd_card.h>
#include <drivers/mmcsd_core.h>
#include <drivers/mmcsd_host.h>
#ifdef RT_USING_DFS
    #include "dfs_file.h"
#endif
#ifdef BSP_USING_SDHCI
    #include "drv_sdhci.h"
    int rt_hw_sdmmc_num_init(uint8_t sdhci_num);
#elif BSP_USING_SD_LINE
    #include "drv_sdio.h"
#endif
#ifdef RT_USING_BT
    #include <drivers/bt_device.h>
    bt_err_t app_bt_disable_afh_channels(bt_channel_map_t *map);
#endif

#ifdef BSP_BLE_SIBLES
#ifndef GAP_LE_CHNL_MAP_LEN
    #define GAP_LE_CHNL_MAP_LEN   (0x05)     /**< LE Channel map length. */
#endif
#define BT_CHNL_MAX_BAND    5
/* Local typedef aligned with bf0_ble_gap.c expectations */
typedef struct
{
    uint8_t channel_map[GAP_LE_CHNL_MAP_LEN];
} ble_gap_update_channel_map_t;
/* Forward declare BLE API */
uint8_t ble_gap_update_channel_map(ble_gap_update_channel_map_t *map);
#endif
void BSP_GPIO_Set(int pin, int val, int is_porta);
static void rt_wifi_set_flag(void *arg, uint32_t flag);
#if !SKW_BOOT_MODE_RAM
    /*
    * 文件模式下，用于分段读取 Wi‑Fi 固件：
    * - wifi_fw_total_size  保存固件总长度；
    * - wifi_fw_fd          当前打开的固件文件；
    * - wifi_fw_fd_opened   标记 fd 是否有效；
    * - wifi_fw_buf         每次从文件中读取到该缓存，再返回指针给上层。
    */
    static struct dfs_fd wifi_fw_fd;
    static rt_size_t      wifi_fw_total_size = 0;
    static rt_uint8_t     wifi_fw_fd_opened  = 0;
    /* 按 4KB 为主的块大小，和 RAM 模式的分段逻辑一致 */
    static char           wifi_fw_buf[4 * 1024];
#endif

/**
 * @brief Local WiFi control callback context.
 *
 * Stored in `device->user_data` after device open, holding suspend/resume/
 * detect-sleep callbacks and their arguments for low-power and wakeup flow.
 */
static struct wifi_ctl wifi_cb_ctx;

/**
 * @brief Get MMC/SD subsystem ready state.
 * @return Non-zero if initialized and ready; 0 if not ready.
 */
uint8_t mmcsd_get_stat(void);

/**
 * @brief Set MMC/SD subsystem ready state flag.
 * @param stat 0 for not ready; non-zero for ready.
 */
void mmcsd_set_stat(uint8_t stat);

#ifdef BSP_USING_SD_LINE
__weak int rt_hw_sdio_deinit(void)
{
    return 0;
}
#endif
void rt_wifi_sdio_init(void)
{
    switch (WIFI_SDIO_NUM)
    {
    case WIFI_SDIO_SDHCI1:
#ifdef BSP_USING_SDHCI1
        rt_hw_sdmmc_num_init(0);
#endif
        break;
    case WIFI_SDIO_SDHCI2:
#ifdef BSP_USING_SDHCI2
        rt_hw_sdmmc_num_init(1);
#endif
        break;
    case WIFI_SDIO_LINE:
#ifdef BSP_USING_SD_LINE
        rt_hw_sdio_init();
#endif
        break;
    default:
        break;
    }
}
void rt_wifi_sdio_deinit(void)
{
    switch (WIFI_SDIO_NUM)
    {
    case WIFI_SDIO_SDHCI1:
#ifdef BSP_USING_SDHCI1
        rt_hw_sdmmc_deinit(0);
#endif
        break;
    case WIFI_SDIO_SDHCI2:
#ifdef BSP_USING_SDHCI2
        rt_hw_sdmmc_deinit(1);
#endif
        break;
    case WIFI_SDIO_LINE:
#ifdef BSP_USING_SD_LINE
        rt_hw_sdmmc_deinit();
#endif
        break;
    default:
        break;
    }
}
__weak void BSP_WIFI_PowerUp()
{
}
__weak void BSP_WIFI_PowerDown()
{
}
void rt_wifi_power(uint8_t power)
{
    rt_wifi_set_flag(NULL, power ? WIFI_DEVICE_INITING : WIFI_DEVICE_IDLE);
    if (power)
    {
        BSP_WIFI_PowerUp();
    }
    else
    {
        BSP_WIFI_PowerDown();
    }
    rt_thread_mdelay(200);
}
/**
 * @brief Poll SDIO/MMC subsystem until ready.
 *
 * Waits up to ~5 seconds (50 * 100ms) and checks readiness via
 * `mmcsd_get_stat()` on each iteration.
 *
 * @return 0 on success (ready); -1 on timeout/failure.
 */
int rt_check_sdio_ready(void)
{
    int timeout = 50; /* max 5s wait for mmcsd */
    while (timeout--)
    {
        if (mmcsd_get_stat())
        {
            break;
        }
        rt_thread_mdelay(100);
    }
    if (timeout <= 0)
    {
        rt_kprintf("mmcsd init fail\n");
        rt_wifi_power(0);
        rt_wifi_sdio_deinit();
        return -1;
    }
    return 0;
}

static void rt_wifi_set_flag(void *arg, uint32_t flag)
{
    rt_device_t device = rt_device_find(SKW_DEV_NAME);
    if (device)
    {
        struct wifi_ctl *ctl = (struct wifi_ctl *)device->user_data;
        ctl->flag_t = flag;
    }
}

uint32_t rt_wifi_get_flag(void)
{
    rt_device_t device = rt_device_find(SKW_DEV_NAME);
    if (device)
    {
        struct wifi_ctl *ctl = (struct wifi_ctl *)device->user_data;
        return ctl->flag_t;
    }
    return 0;
}

/**
 * @brief Suspend WiFi (enter low-power).
 *
 * Powers off WiFi and the wakeup output GPIO, and deinitializes the SDMMC
 * controller.
 *
 * @return Always returns 0.
 */
int rt_wifi_suspend(void)
{
    rt_wifi_power(0);
    rt_wifi_sdio_deinit();
    return 0;
}
/**
 * @brief Resume WiFi (exit low-power).
 *
 * Re-enables power and wakeup output, clears MMC/SD ready state, reinitializes
 * SDMMC, and waits for SDIO subsystem readiness.
 *
 * @return 0 on success; -1 if SDIO initialization fails.
 */
int rt_wifi_resume(void)
{
    rt_kprintf("%s %d\n", __func__, __LINE__);
    rt_wifi_power(1);
    mmcsd_set_stat(0);
    rt_wifi_sdio_init();
    if (rt_check_sdio_ready() != 0)
    {
        rt_kprintf("sdio init fail\n");
        return -1;
    }
    return 0;
}

/**
 * @brief Sleep-wakeup handshake callback.
 *
 * If `WIFI_WAKEUP_IN_PIN` is low, sends a short pulse on
 * `WIFI_WAKEUP_OUT_PIN` and polls within a limited number of attempts until
 * the input returns high.
 */
void wifi_resume_callback(void)
{
    uint16_t timer_t = 1000;
    if (rt_pin_read(WIFI_WAKEUP_IN_PIN) == 0)
    {
        int pin_f = WIFI_WAKEUP_OUT_PIN < GPIO1_PIN_NUM ? 1 : 0;
        BSP_GPIO_Set(pin_f ? WIFI_WAKEUP_OUT_PIN : (WIFI_WAKEUP_OUT_PIN - GPIO1_PIN_NUM), 0, pin_f);
        HAL_Delay_us_(10);
        BSP_GPIO_Set(pin_f ? WIFI_WAKEUP_OUT_PIN : (WIFI_WAKEUP_OUT_PIN - GPIO1_PIN_NUM), 1, pin_f);
        while (!rt_pin_read(WIFI_WAKEUP_IN_PIN) && timer_t--)
        {
            HAL_Delay_us_(10);
        }
    }
}
static int wifi_set_bt_channel_map_adapter(void *arg, int wifi_ch)
{
#ifdef RT_USING_BT
    if (wifi_ch <= 0) return -1;
    if (wifi_ch > 14) return 0;
    int f_center;
    if (wifi_ch == 14)
        f_center = 2484;
    else
        f_center = 2412 + 5 * (wifi_ch - 1);
    int f_lo = f_center - BT_CHNL_MAX_BAND;
    int f_hi = f_center + BT_CHNL_MAX_BAND;

    bt_channel_map_t map;
    map.count = 0;

    /* Iterate BT BR/EDR channels 0..78 */
    for (int n = 0; n <= 78 && map.count < BT_CHANNEL_MAP_BYTES; ++n)
    {
        int f_bt = 2402 + n;
        if (f_bt >= f_lo && f_bt <= f_hi)
        {
            map.channel_map[map.count++] = (uint8_t)n;
        }
    }

    if (map.count > 0)
    {
        (void)app_bt_disable_afh_channels(&map);
        rt_kprintf("AFH: disabled %d BT channels overlapping WiFi ch %d (%.1f MHz)\n",
                   map.count, wifi_ch, (float)f_center);
    }
#endif
    return 0;
}
static int wifi_set_ble_channel_map_adapter(void *arg, int wifi_ch)
{
#ifdef BSP_BLE_SIBLES

    if (wifi_ch <= 0) return -1;
    if (wifi_ch > 14) return 0;
    int f_center;
    if (wifi_ch == 14)
        f_center = 2484;
    else
        f_center = 2412 + 5 * (wifi_ch - 1);

    /* Compute nearest BLE data channel index n in [0..36] against 2404+2*n */
    int n = (int)((f_center - 2404) / 2); /* floor */
    /* choose closer between n and n+1 */
    int f_n = 2404 + 2 * n;
    int f_np1 = 2404 + 2 * (n + 1);
    if (abs(f_np1 - f_center) < abs(f_n - f_center))
        n = n + 1;
    if (n < 0)
        n = 0;
    if (n > 36)
        n = 36;

    ble_gap_update_channel_map_t map;
    /* default enable all channels */
    for (int i = 0; i < GAP_LE_CHNL_MAP_LEN; ++i)
        map.channel_map[i] = 0xFF;
    /* clear only the target BLE data channel bit */
    int byte = n / 8;
    int bit = n % 8;
    map.channel_map[byte] &= (uint8_t)(~(1u << bit));

    (void)ble_gap_update_channel_map(&map);
    rt_kprintf("BLE CHMAP: disabled data ch %d due to WiFi ch %d (%.1f MHz)\n",
               n, wifi_ch, (float)f_center);
#endif
    return 0;
}
/* Adapters to match void (*)(void*) signature */
/**
 * @brief Suspend adapter (matches `void (*)(void*)`).
 * @param arg Unused.
 */
static void wifi_suspend_adapter(void *arg)
{
    rt_wifi_suspend();
}

/**
 * @brief Resume adapter (matches `void (*)(void*)`).
 * @param arg Unused.
 */
static int wifi_resume_adapter(void *arg)
{
    return rt_wifi_resume();
}
/**
 * @brief Detect sleep adapter (matches `void (*)(void*)`).
 * @param arg Unused.
 */
static void wifi_detect_slp_adapter(void *arg)
{
    wifi_resume_callback();
}
static int wifi_get_fwk_maxsize_adapter(void *arg, char *boot_mode)
{
    struct stat boot_file_stat = {0};
    int size = 0;
    if (!rt_strcmp(boot_mode, "dram"))
    {
#if SKW_BOOT_MODE_RAM
        size = sizeof(boot_dram_buff);
#else
        if (dfs_file_stat(WIFI_DRAM_PATH, &boot_file_stat) == 0)
        {
            size = boot_file_stat.st_size;
            /* 重新打开对应文件，供后续分段读取使用 */
            if (wifi_fw_fd_opened)
            {
                dfs_file_close(&wifi_fw_fd);
                wifi_fw_fd_opened = 0;
            }
            if (dfs_file_open(&wifi_fw_fd, WIFI_DRAM_PATH, O_RDONLY) == 0)
            {
                wifi_fw_fd_opened = 1;
                wifi_fw_total_size = boot_file_stat.st_size;
                rt_kprintf("fwk dram open file success, size=%d\n", size);
            }
            else
            {
                rt_kprintf("fwk dram open file failed\n");
                size = 0;
            }
        }
#endif
    }
    else if (!rt_strcmp(boot_mode, "iram"))
    {
#if SKW_BOOT_MODE_RAM
        size = sizeof(boot_iram_buff);
#else
        if (dfs_file_stat(WIFI_IRAM_PATH, &boot_file_stat) == 0)
        {
            size = boot_file_stat.st_size;
            if (wifi_fw_fd_opened)
            {
                dfs_file_close(&wifi_fw_fd);
                wifi_fw_fd_opened = 0;
            }
            if (dfs_file_open(&wifi_fw_fd, WIFI_IRAM_PATH, O_RDONLY) == 0)
            {
                wifi_fw_fd_opened = 1;
                wifi_fw_total_size = boot_file_stat.st_size;
                rt_kprintf("fwk iram open file success, size=%d\n", size);
            }
            else
            {
                rt_kprintf("fwk iram open file failed\n");
                size = 0;
            }
        }
#endif
    }
    else if (!rt_strcmp(boot_mode, "calibration"))
    {
#if SKW_BOOT_MODE_RAM
        size = sizeof(seekwave_buff);
#else
        if (dfs_file_stat(WIFI_CALIBRATION_PATH, &boot_file_stat) == 0)
        {
            size = boot_file_stat.st_size;
            if (wifi_fw_fd_opened)
            {
                dfs_file_close(&wifi_fw_fd);
                wifi_fw_fd_opened = 0;
            }
            if (dfs_file_open(&wifi_fw_fd, WIFI_CALIBRATION_PATH, O_RDONLY) == 0)
            {
                wifi_fw_fd_opened = 1;
                wifi_fw_total_size = boot_file_stat.st_size;
                rt_kprintf("fwk calibration open file success, size=%d\n", size);
            }
            else
            {
                rt_kprintf("fwk calibration open file failed\n");
                size = 0;
            }
        }
#endif
    }
    rt_kprintf("fwk maxsize %s: %d\n", boot_mode, size);
    return size;
}
static int wifi_get_fwk_code_adapter(void *arg, char *boot_mode, uint32_t lseek)
{
    char **out_buf = (char **)arg;
    int size = 0;

    if (!out_buf || !boot_mode)
        return 0;

#if SKW_BOOT_MODE_RAM
    /*
     * 从内存数组分段获取 Wi‑Fi 固件：
     *  - 正常情况下每次返回 4KB 数据；
     *  - 如果剩余数据不足 4KB，则按 512 字节为步长继续分段；
     *  - 最后一小段如果不足 512 字节，则一次性返回剩余长度。
     *
     * 该函数在 skw_sdio_main 中会被循环调用，
     * 调用方通过累计返回的 size 直至达到 maxsize 为止。
     */

    /* 为每种 boot_mode 维护独立 offset */
    static uint32_t dram_offset = 0;
    static uint32_t iram_offset = 0;
    static uint32_t calib_offset = 0;
    /* 为 RAM 模式维护一块当前 chunk 的缓冲区，每次调用按 chunk 重新分配并拷贝数据返回 */
    static unsigned char *chunk_buf = RT_NULL;
    static uint32_t       chunk_buf_size = 0;

    const unsigned char *src = RT_NULL;
    uint32_t total_size = 0;
    uint32_t *offset = RT_NULL;
    rt_bool_t calib_mode = RT_FALSE;

    if (!rt_strcmp(boot_mode, "dram"))
    {
        src        = boot_dram_buff;
        total_size = BOOT_DRAM_SIZE;
        offset     = &dram_offset;
    }
    else if (!rt_strcmp(boot_mode, "iram"))
    {
        src        = boot_iram_buff;
        total_size = BOOT_IRAM_SIZE;
        offset     = &iram_offset;
    }
    else if (!rt_strcmp(boot_mode, "calibration"))
    {
        src        = seekwave_buff;
        total_size = SEEKWAVE_SIZE;
        offset     = &calib_offset;
        calib_mode = RT_TRUE;
    }

    if (!src || !offset)
    {
        *out_buf = RT_NULL;
        return 0;
    }

    /* 所有数据已经发送完成，复位 offset，并释放上一次的 chunk 缓冲 */
    if (*offset >= total_size)
    {
        if (chunk_buf != RT_NULL)
        {
            rt_free(chunk_buf);
            chunk_buf = RT_NULL;
            chunk_buf_size = 0;
        }
        *offset = 0;
        *out_buf = RT_NULL;
        return 0;
    }

    uint32_t remain = total_size - *offset;
    uint32_t chunk  = 0;

    if (calib_mode)
    {
        /* 校准数据结构 skw_calib_param.data 只有 512 字节，
         * 这里严格按 512B 为上限分段，避免越界。 */
        if (remain >= 512)
            chunk = 512;
        else
            chunk = remain;
    }
    else
    {
        if (remain >= 4 * 1024)
        {
            chunk = 4 * 1024;          /* 优先 4KB 分段 */
        }
        else if (remain >= 512)
        {
            chunk = 512;               /* 剩余不足 4KB，则每次 512 字节 */
        }
        else
        {
            chunk = remain;            /* 最后一小段不足 512 字节 */
        }
    }

    /* 为当前 chunk 分配缓冲：按需扩容，避免每次都重新 malloc 不同大小 */
    if (chunk_buf_size < chunk)
    {
        if (chunk_buf != RT_NULL)
        {
            rt_free(chunk_buf);
            chunk_buf = RT_NULL;
            chunk_buf_size = 0;
        }

        chunk_buf = (unsigned char *)rt_malloc(chunk);
        if (chunk_buf == RT_NULL)
        {
            rt_kprintf("fwk malloc chunk failed, size=%u\n", (unsigned int)chunk);
            *out_buf = RT_NULL;
            return 0;
        }
        chunk_buf_size = chunk;
    }

    rt_memcpy(chunk_buf, src + *offset, chunk);

    *out_buf = (char *)chunk_buf;
    *offset += chunk;
    size = (int)chunk;
#else
    /*
     * 从文件系统分段读取 Wi‑Fi 固件：
     *  - wifi_get_fwk_maxsize_adapter 已经根据 boot_mode 打开文件并记录总长度；
     *  - 这里根据 lseek 计算剩余长度，并按 4KB / 512B / 剩余 的规则切分；
     *  - 读取到静态缓存 wifi_fw_buf 中，再把指针返回给上层循环发送。
     */
    if (!wifi_fw_fd_opened || wifi_fw_total_size == 0)
    {
        rt_kprintf("fwk file not opened, mode=%s\n", boot_mode);
        *out_buf = RT_NULL;
        return 0;
    }

    if (lseek >= wifi_fw_total_size)
    {
        /* 读完后复位状态，方便下一次下载 */
        dfs_file_close(&wifi_fw_fd);
        wifi_fw_fd_opened  = 0;
        wifi_fw_total_size = 0;
        *out_buf = RT_NULL;
        return 0;
    }

    uint32_t remain = wifi_fw_total_size - lseek;
    uint32_t chunk  = 0;

    if (!rt_strcmp(boot_mode, "calibration"))
    {
        /* 校准文件同样按 512B 上限分段，便于直接拷贝到
         * skw_calib_param.data[512] 中使用。 */
        if (remain >= 512)
            chunk = 512;
        else
            chunk = remain;
    }
    else
    {
        if (remain >= 4 * 1024)
        {
            chunk = 4 * 1024;
        }
        else if (remain >= 512)
        {
            chunk = 512;
        }
        else
        {
            chunk = remain;
        }
    }

    if (dfs_file_lseek(&wifi_fw_fd, lseek) < 0)
    {
        rt_kprintf("fwk lseek failed, off=%u\n", (unsigned int)lseek);
        *out_buf = RT_NULL;
        return 0;
    }

    int read_len = dfs_file_read(&wifi_fw_fd, wifi_fw_buf, chunk);
    if (read_len != (int)chunk)
    {
        rt_kprintf("fwk read failed, want=%u, read=%d\n", (unsigned int)chunk, read_len);
        *out_buf = RT_NULL;
        return 0;
    }

    *out_buf = wifi_fw_buf;
    size = read_len;
#endif
    return size;
}

static int wifi_get_ram_dump_path_adapter(void *arg, char *path_buf)
{
#ifdef SKW_WIFI_DEBUG
    //检测WIFI_DEBUG_PATH路径是否存在，若存在则返回该路径；不存在就创建这个目录
    extern int mkdir(const char *path, mode_t mode);
    int ret = mkdir(WIFI_DEBUG_PATH, 0);
    if (ret == 0 || errno == -EEXIST)
    {
        rt_kprintf("dir %s ready\n", WIFI_DEBUG_PATH);  // 已存在或创建成功都算 ok
    }
    else
    {
        rt_kprintf("create dir %s failed, errno=%d\n", WIFI_DEBUG_PATH, errno);
        return -1;
    }
    rt_memcpy(path_buf, WIFI_DEBUG_PATH, rt_strlen(WIFI_DEBUG_PATH));
    rt_kprintf("get ram dmup path: %s\n", path_buf);
    return 0;
#endif
    return -1;
}
static int wifi_fwk_file_stat(void)
{
    int ret = 0;
#if !SKW_BOOT_MODE_RAM
    struct stat boot_file_stat = {0};
    if (dfs_file_stat(WIFI_DRAM_PATH, &boot_file_stat) != 0)
    {
        rt_kprintf("stat %s failed\n", WIFI_DRAM_PATH);
        ret |= 1;
    }
    if (dfs_file_stat(WIFI_IRAM_PATH, &boot_file_stat) != 0)
    {
        rt_kprintf("stat %s failed\n", WIFI_IRAM_PATH);
        ret |= 2;
    }
    if (dfs_file_stat(WIFI_CALIBRATION_PATH, &boot_file_stat) != 0)
    {
        rt_kprintf("stat %s failed\n", WIFI_CALIBRATION_PATH);
        ret |= 4;
    }
#endif
    return ret;
}
/**
 * @brief WiFi device initialization (application init stage).
 *
 * - Find and open the device named by `SKW_DEV_NAME`.
 * - Register suspend/resume/detect-sleep callbacks in `device->user_data`.
 * - Initialize WLAN management.
 * - Configure wakeup IRQ.
 *
 * @return 0 on success; -1 if device open fails.
 */
int rt_wifi_init(void)
{
    rt_device_t device = rt_device_find(SKW_DEV_NAME);
    if (device != RT_NULL)
    {
        rt_kprintf("wifi device has been registered\n");
        if (rt_device_open(device, RT_DEVICE_FLAG_RDWR) != RT_EOK)
        {
            rt_kprintf("wifi device open failed\n");
            return -1;
        }
        /* Register suspend/resume callbacks in device user_data */
        struct wifi_ctl *ctl = (struct wifi_ctl *)device->user_data;
        if (ctl == RT_NULL)
        {
            device->user_data = &wifi_cb_ctx;
            ctl = &wifi_cb_ctx;
        }
        ctl->suspend.cb = wifi_suspend_adapter;
        ctl->suspend.arg = RT_NULL;
        ctl->resume.cb = wifi_resume_adapter;
        ctl->resume.arg = RT_NULL;
        ctl->detect_slp.cb = wifi_detect_slp_adapter;
        ctl->detect_slp.arg = RT_NULL;
        ctl->get_fwk_code.cb = wifi_get_fwk_code_adapter;
        ctl->get_fwk_code.arg = RT_NULL;
        ctl->get_fwk_maxsize.cb = wifi_get_fwk_maxsize_adapter;
        ctl->get_fwk_maxsize.arg = RT_NULL;
        ctl->get_ram_dump_path.cb = wifi_get_ram_dump_path_adapter;
        ctl->get_ram_dump_path.arg = RT_NULL;
        ctl->set_channel_map.bt_cb = wifi_set_bt_channel_map_adapter;
        ctl->set_channel_map.ble_cb = wifi_set_ble_channel_map_adapter;
        ctl->set_channel_map.arg = RT_NULL;
        ctl->flag_t = WIFI_DEVICE_NULL;
        ctl->band_2g_5g = WIFI_BOAND_2G_5G;
    }
    if (wifi_fwk_file_stat() != 0)
    {
        rt_kprintf("wifi fwk file stat failed\n");
        return -1;
    }
    swt6621s_wlan_mgnt_init();
    return 0;
}

INIT_APP_EXPORT(rt_wifi_init);