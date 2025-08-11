#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "littlevgl2rtt.h"
#include "lv_ex_data.h"
#include "lv_examples.h"
#include "stdio.h"


#if defined(RT_USING_DFS) && (RT_USING_SPI_MSD)
#include "spi_msd.h"
#include "dfs_file.h"

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
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);

#endif

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t ms;

    /* init littlevGL */
    ret = littlevgl2rtt_init("lcd");
    if (ret != RT_EOK)
    {
        return ret;
    }
    lv_ex_data_pool_init();

    /* add example code here.
       example code locates in folder external/lvgl/examples */
    lv_example_scroll_1();
    //lv_example_grid_1();
    while (1)
    {
        ms = lv_task_handler();
        rt_thread_mdelay(ms);
    }
    return RT_EOK;

}
