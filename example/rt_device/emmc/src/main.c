/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "dfs_file.h"
#include "drivers/mmcsd_core.h"
#include "dfs_posix.h"

/* User code start from here --------------------------------------------------------*/
#ifndef FS_REGION_START_ADDR
    #error "Need to define file system start address!"
#else

    #define FS_CODE "code"
    #define FS_CODE_OFFSET  0X00001000
    #define FS_CODE_LEN     20480*1024

    #define FS_ROOT "root"
    #define FS_ROOT_OFFSET  0X00081000
    #define FS_ROOT_LEN     48*1024*1024  //48M

    #define FS_MSIC "misc"
    #define FS_MSIC_OFFSET  0X00f81000
    #define FS_MSIC_LEN     500*1024*1024 //500MB
#endif

uint8_t mnt_test = 0;
int mnt_init(void)
{
    uint16_t wait_ticks = 400; /* 8s: 400 * 20ms */
    rt_device_t sd_dev = RT_NULL;

    rt_kprintf("wait sd0 device ready...\n");
    while (wait_ticks--)
    {
        rt_thread_mdelay(20);
        sd_dev = rt_device_find("sd0");
        if (sd_dev)
        {
            rt_kprintf("sd0 device ready\n");
            break;
        }
    }

    rt_mmcsd_blk_device_create("sd0", FS_CODE, FS_CODE_OFFSET >> 9, FS_CODE_LEN >> 9);
    rt_mmcsd_blk_device_create("sd0", FS_ROOT, FS_ROOT_OFFSET >> 9, FS_ROOT_LEN >> 9);
    rt_mmcsd_blk_device_create("sd0", FS_MSIC, FS_MSIC_OFFSET >> 9, FS_MSIC_LEN >> 9);
    if (dfs_mount(FS_ROOT, "/", "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", FS_ROOT) == 0)//Format file system
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(FS_ROOT, "/", "elm", 0, 0) == 0)
                rt_kprintf("mount fs on flash success\n");
            else
                rt_kprintf("mount to fs on flash fail\n");
        }
        else
            rt_kprintf("dfs_mkfs elm flash fail\n");
    }
    mkdir("/misc", 0);
    if (dfs_mount(FS_MSIC, "/misc", "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to FS_MSIC success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        rt_kprintf("mount fs on flash to FS_MISC fail\n");
        if (dfs_mkfs("elm", FS_MSIC) == 0)//Format file system
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");

            if (dfs_mount(FS_MSIC, "/misc", "elm", 0, 0) == 0)
                rt_kprintf("mount fs on flash success\n");
            else
                rt_kprintf("mount to fs on flash fail err=%d\n", rt_get_errno());
        }
        else
            rt_kprintf("dfs_mkfs elm flash fail\n");
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);

#if defined(BSP_USING_SDIO)
#include "bf0_hal_aon.h"

#define SDIO_TEST_LEN 512
void cmd_fs_write_t(char *path, int num)
{
    struct dfs_fd fd_test_sd;
    uint32_t open_time = 0, end_time = 0;
    float test_time = 0.0;
    float speed_test = 0.0;
    char *buff = rt_malloc(SDIO_TEST_LEN);
    memset(buff, 0x55, SDIO_TEST_LEN);
    uint32_t write_num = num;
    uint32_t write_byt = write_num * SDIO_TEST_LEN * 8;
    if (dfs_file_open(&fd_test_sd, path, O_RDWR | O_CREAT | O_TRUNC) == 0)
    {
        open_time = HAL_GTIMER_READ();
        while (write_num--)
        {
            dfs_file_write(&fd_test_sd, buff, SDIO_TEST_LEN);
        }
        end_time = HAL_GTIMER_READ();
    }
    dfs_file_close(&fd_test_sd);
    test_time = ((end_time - open_time) / HAL_LPTIM_GetFreq()) * 1000 * 1000;
    speed_test = write_byt / test_time;
    rt_kprintf("%s path=%s num=%d MBts testtime=%.6lfuS,speed_test=%.6lfMb/s\n", __func__, path, num, test_time, speed_test);
    rt_free(buff);
}

void cmd_fs_write(int argc, char **argv)
{
    cmd_fs_write_t(argv[1], atoi(argv[2]));

}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_fs_write, __cmd_fs_write, test write speed);

void cmd_fs_read_t(char *path, int num)
{
    struct dfs_fd fd_read;
    uint32_t open_time = 0, end_time = 0;
    float test_time = 0.0;
    float speed_test = 0.0;
    char *buff = rt_malloc(SDIO_TEST_LEN);
    uint32_t read_num = num;
    uint32_t read_byt = read_num * SDIO_TEST_LEN * 8;
    rt_memset(buff, 0, SDIO_TEST_LEN);
    if (dfs_file_open(&fd_read, path, O_RDONLY) == 0)
    {
        open_time = HAL_GTIMER_READ();
        while (read_num)
        {
            dfs_file_read(&fd_read, buff, SDIO_TEST_LEN);
            read_num--;
        }
        end_time = HAL_GTIMER_READ();
    }
    dfs_file_close(&fd_read);
    test_time = ((end_time - open_time) / HAL_LPTIM_GetFreq()) * 1000 * 1000;
    speed_test = read_byt / test_time;
    rt_kprintf("%s  path=%s num=%d MBts testtime=%.6lfuS,speed_test=%.6lfMb/s\n", __func__, path, num, test_time, speed_test);
    rt_free(buff);
}

void cmd_fs_read(int argc, char **argv)
{
    cmd_fs_read_t(argv[1], atoi(argv[2]));
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_fs_read, __cmd_fs_read, test read speed);
void cmd_fs_read_code_t(char *path, int num)
{
    rt_size_t len = 0;
    char *buff = rt_malloc(SDIO_TEST_LEN);
    uint32_t read_num = num;
    uint32_t read_byt = read_num * SDIO_TEST_LEN * 8;
    rt_memset(buff, 0, SDIO_TEST_LEN);
    rt_device_t code_dev = rt_device_find("code");
    RT_ASSERT(code_dev);
    rt_err_t ret = rt_device_open(code_dev, RT_DEVICE_FLAG_RDWR);
    uint32_t open_time = HAL_GTIMER_READ();
    while (read_num)
    {
        rt_device_read(code_dev, 0, buff, (SDIO_TEST_LEN) >> 9);
        read_num--;
    }
    uint32_t end_time = HAL_GTIMER_READ();
    rt_device_close(code_dev);
    float test_time = ((end_time - open_time) / HAL_LPTIM_GetFreq()) * 1000 * 1000;
    float speed_test = read_byt / test_time;
    rt_kprintf("%s path=%s num=%d MBts testtime=%.6lfuS,speed_test=%.6lfMb/s\n", __func__, path, num, test_time, speed_test);
    rt_free(buff);
}

void cmd_fs_read_code(int argc, char **argv)
{
    cmd_fs_read_code_t(argv[1], atoi(argv[2]));
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_fs_read_code, __cmd_fs_read_code, test read speed code);
void cmd_fs_write_code_t(char *path, int num)
{
    rt_size_t len = 0;
    char *buff = rt_malloc(SDIO_TEST_LEN);
    uint32_t read_num = num;
    uint32_t read_byt = read_num * SDIO_TEST_LEN * 8;
    rt_memset(buff, 0x55, SDIO_TEST_LEN);
    rt_device_t code_dev = rt_device_find("code");
    RT_ASSERT(code_dev);
    rt_err_t ret = rt_device_open(code_dev, RT_DEVICE_FLAG_RDWR);
    uint32_t open_time = HAL_GTIMER_READ();
    while (read_num)
    {
        rt_device_write(code_dev, 0, buff, (SDIO_TEST_LEN) >> 9);
        read_num--;
    }
    uint32_t end_time = HAL_GTIMER_READ();
    rt_device_close(code_dev);
    float test_time = ((end_time - open_time) / HAL_LPTIM_GetFreq()) * 1000 * 1000;
    float speed_test = read_byt / test_time;
    rt_kprintf("%s path=%s num=%d MBts testtime=%.6lfuS,speed_test=%.6lfMb/s\n", __func__, path, num, test_time, speed_test);
    rt_free(buff);
}

void cmd_fs_write_code(int argc, char **argv)
{
    cmd_fs_write_code_t(argv[1], atoi(argv[2]));

}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_fs_write_code, __cmd_fs_write_code, test write speed code);

#endif
#ifdef RT_USING_PM
static void app_wakeup(void)
{
    uint8_t pin = HAL_HPAON_QueryWakeupPin(hwp_gpio2, 54);
    HPAON_WakeupSrcTypeDef src = pin + HPAON_WAKEUP_SRC_PIN0;//PB54
    HAL_StatusTypeDef status = HAL_HPAON_EnableWakeupSrc(src, AON_PIN_MODE_LOW);
}
static rt_timer_t timer_handler = NULL;
static void emmc_pm_test_read(void *param)
{
    cmd_fs_read_t("/1.txt", 1);
}

static void test_time_pm(void)
{
    timer_handler = rt_timer_create("emmc_timer", emmc_pm_test_read, 0, rt_tick_from_millisecond(2000), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    RT_ASSERT(timer_handler);
    rt_timer_start(timer_handler);
}

#endif

int main(void)
{
#ifdef RT_USING_PM
    app_wakeup();
    cmd_fs_write_t("/1.txt", 2);
    test_time_pm();
#endif

    /* Output a message on console using printf function */
    rt_kprintf("Use help to check emmc file system command!\n");
    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);    // Let system breath.
    }
    return 0;
}
