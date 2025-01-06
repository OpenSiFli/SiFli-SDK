#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"


#ifdef FS_REGION_START_ADDR
#define FS_ROOT "root"
#include "dfs_file.h"
#include "drv_flash.h"
int mnt_init(void)
{
    register_mtd_device(FS_REGION_START_ADDR, FS_REGION_SIZE, FS_ROOT);
    if (dfs_mount(FS_ROOT, "/", "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", FS_ROOT) == 0)
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
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);
#elif !defined(BSP_USING_PC_SIMULATOR)
#error "Need to define file system start address!"
#endif


/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    /* Output a message on console using printf function */
    rt_kprintf("Use help to check file system command!\n");

    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);    // Let system breath.
    }
    return 0;
}

