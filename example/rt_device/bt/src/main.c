/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include "bf0_hal.h"
#include "drv_io.h"
#include "bf0_sibles.h"
#include "rt_bt_app.h"

#define DBG_TAG "bt.app.main"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

void HAL_MspInit(void)
{
    BSP_IO_Init();
}

#if defined(BSP_USING_SPI_NAND) && defined(RT_USING_DFS) && !defined(ZBT)
#include "dfs_file.h"
#include "dfs_posix.h"
#include "drv_flash.h"

#ifndef FS_REGION_START_ADDR
    #error "Need to define file system start address!"
#endif

#define NAND_MTD_NAME "root"

static int mnt_init(void)
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
        rt_kprintf("mount fs on flash to root fail, try mkfs\n");
        if (dfs_mkfs("elm", NAND_MTD_NAME) == 0 &&
                dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
        {
            rt_kprintf("mkfs and mount success\n");
        }
        else
        {
            rt_kprintf("mkfs/mount fs on flash fail\n");
        }
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);
#endif /* BSP_USING_SPI_NAND && RT_USING_DFS && !ZBT */

int main(void)
{
    LOG_I("RT-Thread device-framework BT example");

    if (rt_bt_app_core_init() != RT_EOK)
    {
        /* BT init failed; keep the system running so other components work. */
        LOG_E("BT core init failed, example halted");
    }

    sifli_ble_enable();

    LOG_I("waiting for BT stack ready, use \"bt\" to drive the example");

    while (1)
    {
        rt_thread_mdelay(10000);
    }

    return 0;
}
