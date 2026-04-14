/*
 * Copyright (c) 2022-2024 Nordic Semiconductor ASA
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bap_broadcast_src_api.h"
#include <audio_server.h>
#include <audio_mp3ctrl.h>

#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif
#include "drv_flash.h"

#ifndef FS_REGION_START_ADDR
    #error "Need to define file system start address!"
#endif

#define FS_ROOT "root"
/**
 * @brief Mount fs.
 */
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
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);

static mp3ctrl_handle g_handle1;

#define FILE_NAME "48k.mp3"

int main(void)
{
    int err;

    err = bt_enable(NULL);

    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }

    rt_tick_t tick = rt_tick_get();

    for (int i = 0; i < 5; i++)
    {
        rt_thread_mdelay(1000);
    }
    printk("Bluetooth initialized\n");
    bap_broadcast_src_start();
    printk("\r\ninput audio_src 0 to stop\n\n");
    g_handle1 = mp3ctrl_open(AUDIO_TYPE_LOCAL_MUSIC, FILE_NAME, NULL, NULL);
    if (!g_handle1)
    {
        printk("not find file %s\n", FILE_NAME);
    }
    RT_ASSERT(g_handle1);
    mp3ctrl_ioctl(g_handle1, MP3CTRL_IOCTRL_LOOP_TIMES, 0xFFFFFFFF);
    mp3ctrl_play(g_handle1);
}

__ROM_USED void audio_src(int argc, char **argv)
{
    if (argc < 2)
        return;

    if (argv[1][0] == '0')
    {
        bap_broadcast_src_stop();
        mp3ctrl_close(g_handle1);
        g_handle1 = NULL;
        printk("\r\ninput audio_src 1 to start\n\n");
    }
    else
    {
        bap_broadcast_src_start();
        g_handle1 = mp3ctrl_open(AUDIO_TYPE_LOCAL_MUSIC, FILE_NAME, NULL, NULL);
        if (!g_handle1)
        {
            printk("not find file %s\n", FILE_NAME);
        }
        RT_ASSERT(g_handle1);
        mp3ctrl_ioctl(g_handle1, MP3CTRL_IOCTRL_LOOP_TIMES, 0xFFFFFFFF);
        mp3ctrl_play(g_handle1);
        printk("\r\ninput audio_src 0 to stop\n\n");
    }
}
MSH_CMD_EXPORT(audio_src, audio_src command)