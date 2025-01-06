#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "dfs_file.h"
#include "drv_flash.h"
#include "drivers/usb_device.h"
#include "bf0_hal_pcd.h"


#ifndef FS_REGION_START_ADDR
    #error "Need to define file system start address!"
#else
    #define FS_ROOT "root"
    #define FS_ROOT_PATH "/"
#endif

#define FS_MISC "MISC"
#define FS_MISC_PATH "/misc"
#define FS_MISC_SIZE 1024 * 1024 * 10
#define FS_MISC_START_ADDR FS_REGION_START_ADDR + FS_REGION_SIZE

int mnt_init(void)
{
    //FS_REGION_START_ADDR 文件系统的起始地址
    //FS_REGION_SIZE 该文件系统的分区大小
    //FS_ROOT 文件系统名 注：这里必须要有root分区；
    rt_kprintf("0x%x %d\n", FS_REGION_START_ADDR, FS_REGION_SIZE);
    register_mtd_device(FS_REGION_START_ADDR, FS_REGION_SIZE, FS_ROOT); //注册一个文件系统的device，后续的读写操作最终都是操作该denvice
    //mount文件系统
    //FS_ROOT要mount的device
    // FS_ROOT_PATH, 该文件系统的路径，root分区必须是"/",
    // "elm" 文件系统的类型，fat->"elm"
    if (dfs_mount(FS_ROOT, FS_ROOT_PATH, "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        // 如果是第一次mount，那么该地址很有可能没有文件系统分区信息，因此需要格式化该区域（写入分区的LBR信息）
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", FS_ROOT) == 0)//Format file system
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            //格式化成功后再重新mount文件系统
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

int main(void)
{



    /* Output a message on console using printf function */
    rt_kprintf("Use help to check USB mstorage file system command!\n");
    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);    // Let system breath.
    }
    return 0;
}