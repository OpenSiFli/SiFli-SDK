/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lvgl.h"
#include "rtthread.h"
#include "board.h"
#include "drv_touch.h"
#include "lv_port.h"

typedef struct
{
    rt_device_t dev;
    rt_uint32_t data_cnt;
} lv_touchscreen_t;

static rt_err_t touch_rx_indicate(rt_device_t dev, rt_size_t size)
{
    LV_UNUSED(size);

    if (dev->user_data)
    {
        lv_touchscreen_t *touchscreen = (lv_touchscreen_t *)dev->user_data;

        touchscreen->data_cnt++;
    }

    return RT_EOK;
}

static void touch_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    lv_touchscreen_t *touchscreen = lv_indev_get_driver_data(indev);
    struct touch_message touch_data;

    if (!touchscreen || !touchscreen->dev)
    {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    rt_device_read(touchscreen->dev, 0, &touch_data, 1);

    switch (touch_data.event)
    {
        case TOUCH_EVENT_DOWN:
        case TOUCH_EVENT_MOVE:
            data->state = LV_INDEV_STATE_PRESSED;
            break;

        case TOUCH_EVENT_UP:
        default:
            data->state = LV_INDEV_STATE_RELEASED;
            break;
    }

    data->point.x = touch_data.x;
    data->point.y = touch_data.y;

    if (touchscreen->data_cnt > 0)
    {
        touchscreen->data_cnt--;
    }
    data->continue_reading = (touchscreen->data_cnt > 0);
}

void lv_port_indev_init(void)
{
    rt_device_t dev;
    lv_touchscreen_t *touchscreen;
    lv_indev_t *indev;

    dev = rt_device_find("touch");
    if (!dev)
    {
        LV_LOG_WARN("touch device not found");
        return;
    }

    if (rt_device_open(dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        LV_LOG_WARN("cannot open touch device");
        return;
    }

    touchscreen = lv_malloc_zeroed(sizeof(lv_touchscreen_t));
    if (!touchscreen)
    {
        LV_LOG_ERROR("touchscreen allocation failed");
        rt_device_close(dev);
        return;
    }

    touchscreen->dev = dev;
    dev->rx_indicate = touch_rx_indicate;
    dev->user_data = touchscreen;

    indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_read);
    lv_indev_set_driver_data(indev, touchscreen);

    LV_LOG_USER("touchscreen %s opened", "touch");
}
