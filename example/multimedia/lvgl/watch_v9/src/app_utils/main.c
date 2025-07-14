/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     zylx         first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <bf0_hal.h>
#include <board.h>
#include <string.h>
#include <stdbool.h>
#ifdef RT_USING_MODULE
    #include "dlmodule.h"
    #include "dlfcn.h"
#endif
#ifdef BSP_USING_DFU
    #include "dfu.h"
#endif

#ifdef RWBT_ENABLE
    #include "rwip.h"
    #if (NVDS_SUPPORT)
        #include "sifli_nvds.h"
    #endif
#endif
#ifdef RT_USING_XIP_MODULE
    #include "payment_bin.c"
    #include "wf_dig_bin.c"
    #include "dfs_posix.h"
#endif /* RT_USING_MODULE */

#ifdef BSP_BLE_TIMEC
    #include "bf0_ble_tipc.h"
#endif


#ifdef RT_USING_DFS_MNTTABLE
#include "dfs_fs.h"
const struct dfs_mount_tbl mount_table[] =
{
    {
        .device_name = "flash2",
        .path = "/",
        .filesystemtype = "elm",
        .rwflag = 0,
        .data = 0,
    },
    {
        0,
    },
};
#endif
#if 0//def RT_USING_DFS
#include "dfs_file.h"
#include "dfs_posix.h"
#ifndef BSP_USING_PC_SIMULATOR
    #include "drv_flash.h"
#endif /* !BSP_USING_PC_SIMULATOR */

int auto_mnt_init(void)
{
    char *name[2];

#if defined(RT_USING_DFS_ELMFAT)
    const char *type = "elm";
#endif

#ifdef RT_USING_LITTLE_FS
    const char *type = "lfs";
#endif

#if defined(RT_USING_DFS_UFFS)
    const char *type = "uffs";
#endif

#ifdef PKG_USING_DFS_YAFFS
    const char *type = "yaffs";
#endif


    rt_kprintf("===auto_mnt_init===\n");

    memset(name, 0, sizeof(name));

#ifdef RT_USING_SDIO
    //Waitting for SD Card detection done.
    int sd_state = mmcsd_wait_cd_changed(3000);
    if (MMCSD_HOST_PLUGED == sd_state)
    {
        rt_kprintf("SD-Card plug in\n");
        name[0] = "sd0";
    }
    else
    {
        rt_kprintf("No SD-Card detected, state: %d\n", sd_state);
    }
#endif /* RT_USING_SDIO */


#ifdef FS_ROOT_START_ADDR
    name[1] = "flash0";
    register_mtd_device(FS_ROOT_START_ADDR, FS_ROOT_SIZE, name[1]);
#endif /* FS_ROOT_START_ADDR */



    for (uint32_t i = 0; i < sizeof(name) / sizeof(name[0]); i++)
    {
        if (NULL == name[i]) continue;

        if (dfs_mount(name[i], "/", type, 0, 0) == 0) // fs exist
        {
            rt_kprintf("mount fs on %s to root success\n", name[i]);
            break;
        }
        else
        {
            rt_kprintf("mount fs on %s to root fail\n", name[i]);
        }
    }

    return RT_EOK;
}
#endif /* RT_USING_DFS */


#define BOOT_LOCATION 1
int main(void)
{
    int count = 0;

#ifdef SOC_BF0_LCPU
    env_init();
    //sifli_mbox_init();
#if (NVDS_SUPPORT)
#ifdef RT_USING_PM
#ifdef PM_STANDBY_ENABLE
    g_boot_mode = rt_application_get_power_on_mode();
    if (g_boot_mode == 0)
#endif // PM_STANDBY_ENABLE
#endif // RT_USING_PM
    {
        sifli_nvds_init();
    }
#else
    rwip_init(0);
#endif // NVDS_SUPPORT

#endif

#ifdef BSP_BLE_TIMEC
    ble_tipc_init(true);
#endif

    return RT_EOK;
}

#ifdef RT_USING_MODULE

int mod_load(int argc, char **argv)
{
    if (argc < 2)
        return -1;
    dlopen(argv[1], 0);
    return 0;
}
MSH_CMD_EXPORT(mod_load, Load module);

#ifdef RT_USING_XIP_MODULE

static struct rt_dlmodule *test_module;
int mod_run(int argc, char **argv)
{
    const char *install_path;
    const char *mod_name;

    if (argc < 2)
    {
        rt_kprintf("wrong argument\n");
        return -1;
    }

    mod_name = argv[1];
    if (argc >= 3)
    {
        install_path = argv[2];
    }
    else
    {
        install_path = "/app";
    }

    test_module = dlrun(mod_name, install_path);
    rt_kprintf("run %s:0x%x,%d\n", mod_name, test_module, test_module->nref);
    if (test_module->nref)
    {
        if (test_module->init_func)
        {
            test_module->init_func(test_module);
        }
    }
    return 0;
}
MSH_CMD_EXPORT(mod_run, Run module);

int get_f_phy_addr(int argc, char **argv)
{
    int fid;
    uint32_t addr;
    int res;

    if (argc < 2)
    {
        return -1;
    }
    fid = open(argv[1], O_RDONLY);
    if (fid >= 0)
    {
        uint8_t buf[10];
        res = read(fid, buf, 5);
        res = ioctl(fid, F_GET_PHY_ADDR, &addr);
        close(fid);
        if (res < 0)
        {
            return -1;
        }
        rt_kprintf("addr: 0x%p\n", addr);
    }
    return 0;
}
MSH_CMD_EXPORT(get_f_phy_addr, Get file physical address);
#endif /* RT_USING_XIP_MODULE */


int mod_free(int argc, char **argv)
{
    struct rt_dlmodule *hdl;
    if (argc < 2)
        return -1;

    hdl = dlmodule_find(argv[1]);
    if (hdl != RT_NULL)
        dlclose((void *)hdl);
    return 0;
}
MSH_CMD_EXPORT(mod_free, Free module);

#endif

