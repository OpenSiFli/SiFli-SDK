/**
 * @file main.c
 * @brief BT656 example entry point and camera shell command.
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @brief BT656 example entry point (dpi-hdk-lb57e).
 *
 * Every feature is used through the single shell command `camera`:
 *   camera snapshot [loops] [timeout_ms]   capture and show single frames
 *   camera frame [interval_ms]             continuous frame-by-frame display
 *   camera stream [interval_ms]            continuous stream display (highest frame rate)
 *   camera init                            initialize the decoder
 *   camera stop                            stop continuous display
 *   camera info                            print the current status
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdlib.h>
#include <board.h>

#include "bt656.h"
#include "bt656_show.h"
#include "cpu_usage_profiler.h"

#define DBG_TAG "bt656.demo"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static void camera_usage(void)
{
    rt_kprintf("usage:\n");
    rt_kprintf("  camera snapshot [loops] [timeout_ms]  capture & show N frames (default 1, 1000 ms)\n");
    rt_kprintf("  camera frame [interval_ms]            continuous frame mode (default 0 ms)\n");
    rt_kprintf("  camera stream [interval_ms]           continuous stream mode (default 0 ms)\n");
    rt_kprintf("  camera init                           initialize decoder\n");
    rt_kprintf("  camera stop                           stop continuous display\n");
    rt_kprintf("  camera cpu                            print CPU usage (last ~2 s window)\n");
    rt_kprintf("  camera info                           print camera status\n");
}

static void camera_print_cpu(void)
{
#if defined(USING_CPU_USAGE_PROFILER)
    rt_kprintf("[camera] cpu    : %.2f%% (last ~2 s window)\n",
               (double)cpu_get_usage());
#else
    rt_kprintf("[camera] cpu    : n/a (enable CONFIG_USING_CPU_USAGE_PROFILER)\n");
#endif
}

static void camera_info(void)
{
    const struct bt656_format *info = bt656_get_format();
    rt_uint32_t frames = 0;
    rt_uint32_t fails = 0;
    rt_uint32_t captured = 0;
    rt_uint32_t dropped = 0;
    rt_uint32_t errors = 0;

    if (info == RT_NULL)
    {
        rt_kprintf("[camera] format not configured\n");
        return;
    }

    rt_kprintf("[camera] format : %s (%ux%u, stride %u, %s)\n",
               info->name, info->width, info->height, info->stride,
               (info->interlace != 0U) ? "interlaced field" : "progressive");
    rt_kprintf("[camera] signal : %s (chip id 0x%04x)\n",
               bt656_signal_locked() ? "locked" : "no lock",
               bt656_detect());

    bt656_stats(&captured, &dropped, &errors);
    rt_kprintf("[camera] stats  : captured=%u, dropped=%u, errors=%u\n",
               (unsigned int)captured, (unsigned int)dropped,
               (unsigned int)errors);

    if (camera_preview_running())
    {
        camera_preview_stats(&frames, &fails);

        rt_kprintf("[camera] preview: running, mode=%s, captured=%u, failed=%u\n",
                   (camera_preview_mode() == BT656_PREVIEW_STREAM) ?
                   "stream" : "frame", (unsigned int)frames, (unsigned int)fails);
    }
    else
    {
        rt_kprintf("[camera] preview: stopped\n");
    }

    camera_print_cpu();
}

static int camera_snapshot(int loops, rt_int32_t tmo)
{
    int i;
    int ret;

    /* Single-frame and continuous preview are mutually exclusive */
    ret = camera_preview_stop();
    if (ret != RT_EOK)
    {
        rt_kprintf("[camera] preview stop failed: %d\n", ret);
        return ret;
    }

    if (loops <= 0)
    {
        loops = 1;
    }

    ret = bt656_init();
    if (ret == RT_EOK)
    {
        ret = bt656_start();
    }
    if (ret != RT_EOK)
    {
        rt_kprintf("[camera] start failed: %d\n", ret);
        return ret;
    }

    rt_kprintf("[camera] snapshot: %d frame(s), timeout %d ms\n",
               loops, (int)tmo);

    for (i = 0; i < loops; i++)
    {
        int r = camera_show_once(tmo);

        if (r != RT_EOK)
        {
            rt_kprintf("[camera] snapshot failed: %d\n", r);
            ret = r;
        }
    }

    (void)bt656_stop();
    return ret;
}

static int camera(int argc, char **argv)
{
    const char *sub;

    if (argc < 2)
    {
        camera_usage();
        return RT_EOK;
    }

    sub = argv[1];

    if ((strcmp(sub, "help") == 0) || (strcmp(sub, "-h") == 0))
    {
        camera_usage();
        return RT_EOK;
    }

    if (strcmp(sub, "init") == 0)
    {
        const struct bt656_format *info;
        int ret = bt656_init();

        if (ret != RT_EOK)
        {
            rt_kprintf("[camera] decoder init failed: %d\n", ret);
            return ret;
        }

        info = bt656_get_format();
        rt_kprintf("[camera] ready: %s, signal %s\n",
                   (info != RT_NULL) ? info->name : "unknown",
                   bt656_signal_locked() ? "locked" : "no lock");
        return RT_EOK;
    }

    if (strcmp(sub, "snapshot") == 0)
    {
        int loops = (argc > 2) ? atoi(argv[2]) : 1;
        rt_int32_t tmo = (argc > 3) ? (rt_int32_t)strtoul(argv[3], RT_NULL, 0) : 1000;

        return camera_snapshot(loops, tmo);
    }

    if ((strcmp(sub, "frame") == 0) || (strcmp(sub, "stream") == 0))
    {
        camera_preview_mode_t mode = (strcmp(sub, "stream") == 0) ?
                                     BT656_PREVIEW_STREAM : BT656_PREVIEW_FRAME;
        rt_int32_t interval = (argc > 2) ?
                              (rt_int32_t)strtoul(argv[2], RT_NULL, 0) : 0;
        int ret;

        if (interval < 0)
        {
            interval = 0;
        }

        if (camera_preview_running())
        {
            rt_kprintf("[camera] already running, 'camera stop' first\n");
            return -RT_EBUSY;
        }

        ret = camera_preview_start(mode, interval);
        if (ret != RT_EOK)
        {
            rt_kprintf("[camera] start failed: %d\n", ret);
        }
        return ret;
    }

    if (strcmp(sub, "stop") == 0)
    {
        int ret = camera_preview_stop();

        rt_kprintf("[camera] %s\n", (ret == RT_EOK) ? "stopped" : "stop timeout");
        return ret;
    }

    if (strcmp(sub, "info") == 0)
    {
        camera_info();
        return RT_EOK;
    }

    if (strcmp(sub, "cpu") == 0)
    {
        camera_print_cpu();
        return RT_EOK;
    }

    rt_kprintf("[camera] unknown subcommand '%s'\n", sub);
    camera_usage();
    return -RT_EINVAL;
}
MSH_CMD_EXPORT(camera, camera snapshot / frame / stream / stop / info);

int main(void)
{
    rt_kprintf("\n===== BT656 camera demo (dpi-hdk-lb57e) =====\n");
    camera_usage();

    while (1)
    {
        rt_thread_mdelay(1000);
    }
}
