/**
 * @file bt656_lvgl.c
 * @brief BT656 + LVGL image widget preview example.
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @brief BT656 + LVGL image widget example (optional, not built by default).
 *
 * In the project directory run `sdk.py menuconfig --board=<board>` and enable
 * the top-level option `BT656 + LVGL preview demo` (BT656_DEMO_USING_LVGL in
 * Kconfig.proj). The middleware/lvgl option CONFIG_PKG_USING_LITTLEVGL2RTT=y is
 * required as well.
 *
 * Principle:
 *   The LVGL v8 image widget supports LV_IMG_CF_YUV422_PACKED_UYVY, and the
 *   SDK's LVGL GPU driver hands it to the EPIC hardware compositor, so neither
 *   CPU color conversion nor the show_fb / LCD attach machinery of this example
 *   is needed.
 *
 * Frame ownership:
 *   The lv_timer takes one frame non-blocking every 16 ms, updates
 *   lv_img_dsc_t and then returns the "previous frame" to the driver. The
 *   previous frame has already gone through at least one LVGL refresh (EPIC
 *   completed), so it can be reclaimed safely; the current frame is returned on
 *   the next update.
 *
 * Limitations:
 *   This example supports CVBS only (UYVY, stride == 2*width, no BT.656 timing
 *   codes). AHD buffers still carry the timing codes, and lv_img_dsc_t has no
 *   stride field, so they need to be rearranged by the driver before they can
 *   be used with lv_img_set_src.
 *
 * Note:
 *   This example creates its own LVGL host thread and calls
 *   littlevgl2rtt_init("lcd"); if the project already has an LVGL thread, move
 *   the cam_timer / dsc update part there instead of initializing twice.
 *   Do not run it together with `camera stream` (both need the lcd device
 *   exclusively).
 */

#include "rtconfig.h"

#if defined(BT656_DEMO_USING_LVGL)

#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>

#include "lvgl.h"
#include "littlevgl2rtt.h"

#include "bt656.h"
#include "bt656_show.h"

#define DBG_TAG "bt656.lvgl"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#if (LVGL_VERSION_MAJOR != 8)
    #error "bt656_lvgl demo requires LVGL v8"
#endif

/** @brief Image widget refresh period (ms) */
#define CAM_POLL_MS     16

static lv_obj_t *cam_img = RT_NULL;
static lv_img_dsc_t cam_dsc;
static struct bt656_frame cam_held;
static rt_bool_t cam_held_valid = RT_FALSE;
static lv_timer_t *cam_timer = RT_NULL;

static volatile rt_bool_t lvgl_run = RT_FALSE;
static rt_thread_t lvgl_thread = RT_NULL;
/** LVGL itself is initialized only once: littlevgl2rtt_init() must not run
 *  again after a stop/start cycle */
static rt_bool_t s_lvgl_inited = RT_FALSE;

static void cam_timer_cb(lv_timer_t *timer)
{
    struct bt656_frame f;

    (void)timer;

    /* Non-blocking take: use the frame if there is one, otherwise keep showing
     * the previous one */
    if (bt656_get(&f, 0) != RT_EOK)
    {
        return;
    }

    /*
     * The LVGL image descriptor has no stride field, so stride = 2 * width is
     * implied. Skip AHD frames that carry timing codes to avoid a garbled
     * display.
     */
    if ((f.field == 0U) || (f.stride != (rt_uint32_t)f.width * 2U))
    {
        static rt_bool_t warned = RT_FALSE;

        if (!warned)
        {
            LOG_W("LVGL demo supports packed CVBS(UYVY) only, frame skipped");
            warned = RT_TRUE;
        }
        bt656_release(&f);
        return;
    }

    /* The data address stays the same while the content changes: make LVGL drop
     * its image cache */
    if (cam_dsc.data != RT_NULL)
    {
        lv_img_cache_invalidate_src(&cam_dsc);
    }

    cam_dsc.header.always_zero = 0;
    cam_dsc.header.cf          = LV_IMG_CF_YUV422_PACKED_UYVY;
    cam_dsc.header.w           = f.width;
    cam_dsc.header.h           = f.height;
    cam_dsc.data               = f.data;
    cam_dsc.data_size          = (uint32_t)f.stride * f.height;

    lv_img_set_src(cam_img, &cam_dsc);

    /*
     * The previous frame has been through at least one refresh (EPIC completed)
     * and can be returned to the driver. The current frame is returned on the
     * next update.
     */
    if (cam_held_valid)
    {
        bt656_release(&cam_held);
    }
    cam_held = f;
    cam_held_valid = RT_TRUE;
}

static void lvgl_thread_entry(void *parameter)
{
    (void)parameter;

    if (!s_lvgl_inited)
    {
        littlevgl2rtt_init("lcd");
        s_lvgl_inited = RT_TRUE;
    }

    cam_img = lv_img_create(lv_scr_act());
    lv_obj_align(cam_img, LV_ALIGN_CENTER, 0, 0);

    cam_timer = lv_timer_create(cam_timer_cb, CAM_POLL_MS, RT_NULL);

    while (lvgl_run)
    {
        lv_task_handler();
        rt_thread_mdelay(5);
    }

    if (cam_timer != RT_NULL)
    {
        lv_timer_del(cam_timer);
        cam_timer = RT_NULL;
    }
    if (cam_img != RT_NULL)
    {
        lv_obj_del(cam_img);
        cam_img = RT_NULL;
    }
    if (cam_held_valid)
    {
        bt656_release(&cam_held);
        cam_held_valid = RT_FALSE;
    }

    lvgl_thread = RT_NULL;
}

static int camera_lvgl(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("usage: camera_lvgl start | stop\n");
        return -RT_EINVAL;
    }

    if (strcmp(argv[1], "start") == 0)
    {
        int ret;

        if (lvgl_run)
        {
            return RT_EOK;
        }
        if (camera_preview_running())
        {
            rt_kprintf("[camera_lvgl] 'camera stop' first\n");
            return -RT_EBUSY;
        }

        ret = bt656_init();
        if (ret == RT_EOK)
        {
            ret = bt656_start();
        }
        if (ret != RT_EOK)
        {
            rt_kprintf("[camera_lvgl] capture start failed: %d\n", ret);
            return ret;
        }

        lvgl_run = RT_TRUE;
        lvgl_thread = rt_thread_create("ark_lvgl", lvgl_thread_entry,
                                       RT_NULL, 8192, 22, 10);
        if (lvgl_thread == RT_NULL)
        {
            lvgl_run = RT_FALSE;
            (void)bt656_stop();
            return -RT_ENOMEM;
        }
        rt_thread_startup(lvgl_thread);
        return RT_EOK;
    }

    if (strcmp(argv[1], "stop") == 0)
    {
        int i;

        if (!lvgl_run && (lvgl_thread == RT_NULL))
        {
            /* LVGL preview was never started: the native preview may own the
             * capture path, so do not stop it here */
            return RT_EOK;
        }

        lvgl_run = RT_FALSE;
        for (i = 0; (i < 60) && (lvgl_thread != RT_NULL); i++)
        {
            rt_thread_mdelay(50);
        }
        if (lvgl_thread != RT_NULL)
        {
            rt_kprintf("[camera_lvgl] thread did not exit in 3s\n");
            return -RT_ETIMEOUT;
        }
        (void)bt656_stop();
        return RT_EOK;
    }

    rt_kprintf("usage: camera_lvgl start | stop\n");
    return -RT_EINVAL;
}
MSH_CMD_EXPORT(camera_lvgl, BT656 LVGL image widget preview);

#endif /* BT656_DEMO_USING_LVGL */
