/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-21     zylx         first version
 */

#include "board.h"

#if 0
    uint8_t ALIGN(4) mpi_test_data[SPI_NAND_PAGE_SIZE];
    uint8_t ALIGN(4) mpi_test_data2[SPI_NAND_PAGE_SIZE];
#endif

void SystemClock_Config(void)
{

}

#ifdef RT_USING_DFS
#include "dfs_file.h"
#include "dfs_posix.h"
#include "drv_flash.h"
#define NAND_MTD_NAME    "mpi3"
int mnt_init(void)
{
    //TODO: how to get base address
    //register_nor_device(FS_ROOT_START_ADDR & (0xFF000000), FS_ROOT_START_ADDR - (FS_ROOT_START_ADDR & (0xFF000000)), FS_ROOT_SIZE, NAND_MTD_NAME);
    register_nand_device(FS_ROOT_START_ADDR & (0xFF000000), FS_ROOT_START_ADDR - (FS_ROOT_START_ADDR & (0xFF000000)), FS_ROOT_SIZE, NAND_MTD_NAME);
    if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        rt_kprintf("mount fs on flash to root fail\n");
#if 1
        if (dfs_mkfs("elm", NAND_MTD_NAME) == 0)
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
                rt_kprintf("mount fs on flash success\n");
            else
                rt_kprintf("mount to fs on flash fail\n");
        }
        else
            rt_kprintf("dfs_mkfs elm flash fail\n");
#endif
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);
#endif /* RT_USING_DFS */


