/*
 * SPDX-FileCopyrightText: 2025-2026 SiFli Technologies(Nanjing) Co., Ltd
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
int mpi2_mnt(void);
uint8_t mnt_test = 0;
int mnt_init(void)
{
    uint16_t sdhci_time = 100;
    while (sdhci_time --)
    {
        rt_thread_mdelay(30);
        uint8_t mmcsd_get_stat(void);
        if (mmcsd_get_stat()) break;
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
    mpi2_mnt();
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
#define SDIO_DATA 0xf0
void cmd_fs_write_t(char *path, int num)
{
    struct dfs_fd fd_test_sd;
    uint32_t open_time = 0, end_time = 0;
    float test_time = 0.0;
    float speed_test = 0.0;
    char *buff = rt_malloc(SDIO_TEST_LEN);
    memset(buff, SDIO_DATA, SDIO_TEST_LEN);
    uint32_t write_num = num;
    uint32_t write_byt = write_num * SDIO_TEST_LEN;
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
    test_time = ((end_time - open_time) / HAL_LPTIM_GetFreq());
    speed_test = write_byt / test_time;
    rt_kprintf("%s path=%s num=%d byte testtime=%.4lfmS,speed_test=%.6lfKB/s\n", __func__, path, write_byt, test_time * 1000, speed_test / 1024);
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
    uint32_t read_byt = read_num * SDIO_TEST_LEN;
    rt_memset(buff, 0, SDIO_TEST_LEN);
    if (dfs_file_open(&fd_read, path, O_RDONLY) == 0)
    {
        open_time = HAL_GTIMER_READ();
        while (read_num)
        {
            dfs_file_read(&fd_read, buff, SDIO_TEST_LEN);
            for (int i = 0; i < SDIO_TEST_LEN; i++)
            {
                if (buff[i] != SDIO_DATA) RT_ASSERT(0);
            }
            read_num--;
        }
        end_time = HAL_GTIMER_READ();
    }
    dfs_file_close(&fd_read);
    test_time = ((end_time - open_time) / HAL_LPTIM_GetFreq());
    speed_test = read_byt / test_time;
    rt_kprintf("%s path=%s num=%d byte testtime=%.4lfmS,speed_test=%.6lfKB/s\n", __func__, path, read_byt, test_time * 1000, speed_test / 1024);
    rt_free(buff);
}

void cmd_fs_read(int argc, char **argv)
{
    cmd_fs_read_t(argv[1], atoi(argv[2]));
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_fs_read, __cmd_fs_read, test read speed);

#endif
#ifdef BSP_USING_SWITCH_MPI2_SDIO
#include "drv_flash.h"
#define MPI2_NOR "mpi2_nor"
#define MPI2_NOR_PATH "mpi2_nor"
int mpi2_mnt(void)
{
    //FS_REGION_START_ADDR 文件系统的起始地址
    //FS_REGION_SIZE 该文件系统的分区大小
    //FS_ROOT 文件系统名 注：这里必须要有root分区；
    rt_kprintf("0x%x %d\n", FS_REGION_START_ADDR, FS_REGION_SIZE);
    register_mtd_device(FS_REGION_START_ADDR, FS_REGION_SIZE, MPI2_NOR); //注册一个文件系统的device，后续的读写操作最终都是操作该denvice
    //register_nor_device(0x12000000, 0x720000, 0x200000, MPI2_NOR);
    //mount文件系统
    //FS_ROOT要mount的device
    // FS_ROOT_PATH, 该文件系统的路径，root分区必须是"/",
    // "elm" 文件系统的类型，fat->"elm"
    mkdir(MPI2_NOR_PATH, 0);
    if (dfs_mount(MPI2_NOR, MPI2_NOR_PATH, "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        //RT_ASSERT(0);
        // 如果是第一次mount，那么该地址很有可能没有文件系统分区信息，因此需要格式化该区域（写入分区的LBR信息）
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", MPI2_NOR) == 0)//Format file system
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            //格式化成功后再重新mount文件系统
            if (dfs_mount(MPI2_NOR, MPI2_NOR_PATH, "elm", 0, 0) == 0)
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

MSH_CMD_EXPORT(mpi2_mnt, mpi2_mnt);
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

