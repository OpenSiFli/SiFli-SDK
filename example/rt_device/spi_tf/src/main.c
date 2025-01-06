#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "dfs_file.h"
#include "spi_msd.h"

#define FS_ROOT "root"
#define FS_ROOT_PATH "/"
#define FS_ROOT_OFFSET  0X00000000
#define FS_ROOT_LEN     500*1024*1024  //500M

#define FS_MSIC "misc"
#define FS_MSIC_PATH "/misc"
#define FS_MSIC_OFFSET  (FS_ROOT_OFFSET + FS_ROOT_LEN)
#define FS_MSIC_LEN     500*1024*1024 //500MB

#define FS_BLOCK_SIZE 0x200

struct rt_device *fal_mtd_msd_device_create(char *name, long offset, long len)
{

    rt_device_t msd = rt_device_find("sd0");
    if (msd == NULL)
    {
        rt_kprintf("Error: the flash device name (sd0) is not found.\n");
        return NULL;
    }
    struct msd_device *msd_dev = (struct msd_device *)msd->user_data;

    struct msd_device *msd_file_dev = (struct msd_device *)rt_malloc(sizeof(struct msd_device));
    if (msd_file_dev)
    {
        msd_file_dev->parent.type        = RT_Device_Class_MTD;
#ifdef RT_USING_DEVICE_OPS
        msd_file_dev->parent.ops        = msd_dev->parent.ops;
#else
        msd_file_dev->parent.init       = msd_dev->parent.init;
        msd_file_dev->parent.open       = msd_dev->parent.open;
        msd_file_dev->parent.close      = msd_dev->parent.close;
        msd_file_dev->parent.read       = msd_dev->parent.read;
        msd_file_dev->parent.write      = msd_dev->parent.write;
        msd_file_dev->parent.control    = msd_dev->parent.control;
#endif
        msd_file_dev->offset            = offset;
        msd_file_dev->spi_device        = msd_dev->spi_device;
        msd_file_dev->geometry.bytes_per_sector = FS_BLOCK_SIZE;
        msd_file_dev->geometry.block_size = FS_BLOCK_SIZE;
        msd_file_dev->geometry.sector_count = len;

        rt_device_register(&msd_file_dev->parent, name,
                           RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE | RT_DEVICE_FLAG_STANDALONE);
        rt_kprintf("fal_mtd_msd_device_create dev:sd0 part:%s offset:0x%x, size:0x%x\n", name, msd_file_dev->offset, msd_file_dev->geometry.sector_count);
        return RT_DEVICE(&msd_file_dev->parent);;
    }
    return NULL;
}

int mnt_init(void)
{
    uint16_t time_out = 100;
    while (time_out --)
    {
        rt_thread_mdelay(30);
        if (rt_device_find("sd0"))
            break;
    }
    fal_mtd_msd_device_create(FS_ROOT, FS_ROOT_OFFSET >> 9, FS_ROOT_LEN >> 9);
    fal_mtd_msd_device_create(FS_MSIC, FS_MSIC_OFFSET >> 9, FS_MSIC_LEN >> 9);
    if (dfs_mount(FS_ROOT, FS_ROOT_PATH, "elm", 0, 0) == 0) // fs exist
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
            {
                rt_kprintf("mount to fs on flash fail\n");
                return RT_ERROR;
            }
        }
        else
            rt_kprintf("dfs_mkfs elm flash fail\n");
    }
    extern int mkdir(const char *path, mode_t mode);
    mkdir(FS_MSIC_PATH, 0);
    if (dfs_mount(FS_MSIC, FS_MSIC_PATH, "elm", 0, 0) == 0) // fs exist
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


#include "bf0_hal_aon.h"
rt_uint32_t *buff_test = (rt_uint32_t *)(0x60000000);

#define SDIO_TEST_LEN 1024 * 1024
void cmd_fs_write_t(char *path, int num)
{
    struct dfs_fd fd_test_sd;
    uint32_t open_time = 0, end_time = 0;
    float test_time = 0.0;
    float speed_test = 0.0;
    //char *buff = app_malloc(SDIO_TEST_LEN);
    memset(buff_test, 0x55, SDIO_TEST_LEN);
    uint32_t write_num = num;
    uint32_t write_byt = write_num * SDIO_TEST_LEN * 8;
    if (dfs_file_open(&fd_test_sd, path, O_RDWR | O_CREAT | O_TRUNC) == 0)
    {
        open_time = HAL_GTIMER_READ();
        while (write_num--)
        {
            dfs_file_write(&fd_test_sd, buff_test, SDIO_TEST_LEN);
        }
        end_time = HAL_GTIMER_READ();
    }
    dfs_file_close(&fd_test_sd);
    test_time = ((end_time - open_time) / HAL_LPTIM_GetFreq()) * 1000 * 1000;
    speed_test = write_byt / test_time;
    rt_kprintf("%s path=%s num=%d b testtime=%.6lfuS,speed_test=%.6lfMb/s\n", __func__, path, num, test_time, speed_test);
    //app_free(buff);

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
    //char *buff = app_malloc(SDIO_TEST_LEN);
    uint32_t read_num = num;
    uint32_t read_byt = read_num * SDIO_TEST_LEN * 8;
    rt_memset(buff_test, 0, SDIO_TEST_LEN);
    if (dfs_file_open(&fd_read, path, O_RDONLY) == 0)
    {
        open_time = HAL_GTIMER_READ();
        while (read_num)
        {
            dfs_file_read(&fd_read, buff_test, SDIO_TEST_LEN);
            read_num--;
        }
        end_time = HAL_GTIMER_READ();
    }
    dfs_file_close(&fd_read);
    test_time = ((end_time - open_time) / HAL_LPTIM_GetFreq()) * 1000 * 1000;
    speed_test = read_byt / test_time;
    rt_kprintf("%s  path=%s num=%d b testtime=%.6lfuS,speed_test=%.6lfMb/s\n", __func__, path, num, test_time, speed_test);
    //app_free(buff);
}

void cmd_fs_read(int argc, char **argv)
{
    cmd_fs_read_t(argv[1], atoi(argv[2]));
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_fs_read, __cmd_fs_read, test read speed);

int cmd_emmc_test_buff(int argc, char **argv)
{
    if (atoi(argv[1]) == 512)
        buff_test = (rt_uint32_t *)(0x60000000);//
    else if (atoi(argv[1]) == 1024)
        buff_test = (rt_uint32_t *)(0x60100000);//1M
    else buff_test = (rt_uint32_t *)(0x60000003);
    rt_kprintf("%s buff_test=%p\n", __func__, buff_test);
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_emmc_test_buff, __cmd_emmc_test_buff, cmd emmc tes buff);

int main(void)
{
    /* Output a message on console using printf function */
    rt_kprintf("Use help to check spi sd file system command!\n");
    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);    // Let system breath.
    }
    return 0;
}

