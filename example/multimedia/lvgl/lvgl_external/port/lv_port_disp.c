/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lvgl.h"
#include "rtthread.h"
#include "board.h"
#include "drv_lcd.h"
#include "lv_port.h"

#define FLUSH_TIMEOUT_MS 5000

/* Draw buffers are sized for the panel selected by the board configuration.
 * PARTIAL rendering: LVGL fills one strip and hands it over while it renders
 * the next one, so the buffer is "screen width x lines per strip". */
#ifndef LCD_HOR_RES_MAX
#define LCD_HOR_RES_MAX 480
#endif
#ifndef LCD_VER_RES_MAX
#define LCD_VER_RES_MAX 480
#endif
#ifndef LV_EXT_FB_LINE_NUM
#define LV_EXT_FB_LINE_NUM LCD_VER_RES_MAX
#endif

#define PORT_BUF_SIZE (LCD_HOR_RES_MAX * LV_EXT_FB_LINE_NUM * \
                       LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_DEFAULT))
/* lv_draw_buf_reshape() sizes a shape as LV_ROUND_UP(stride * h,
 * LV_DRAW_BUF_ALIGN). A buffer allocated with exactly stride * h bytes cannot
 * take that shape back: the reshape fails, the previous (smaller) header stays
 * while buf_area has already moved to the new area, and the renderer runs off
 * the buffer. Round the allocation up to the same alignment. */
#define PORT_BUF_ALLOC_SIZE LV_ROUND_UP(PORT_BUF_SIZE, LV_DRAW_BUF_ALIGN)

static rt_device_t lcd_dev;
static struct rt_device_graphic_info lcd_info;
static struct rt_semaphore flush_sem;
static lv_display_t *disp;
static lv_display_t *flush_disp;
static bool frame_start = true;

ALIGN(LV_DRAW_BUF_ALIGN) static uint8_t draw_buf_1[PORT_BUF_ALLOC_SIZE];
#if defined(LV_EXT_FB_TWO_PARTIAL)
ALIGN(LV_DRAW_BUF_ALIGN) static uint8_t draw_buf_2[PORT_BUF_ALLOC_SIZE];
#endif

static rt_err_t lcd_flush_done(rt_device_t dev, void *buffer)
{
    lv_display_t *d;

    LV_UNUSED(dev);
    LV_UNUSED(buffer);

    d = flush_disp;
    flush_disp = NULL;
    if (d)
    {
        lv_display_flush_ready(d);
    }
    rt_sem_release(&flush_sem);

    return RT_EOK;
}

static void lcd_flush(lv_display_t *d, const lv_area_t *area, uint8_t *px_map)
{
    LCD_AreaDef window;
    uint8_t wait_te;

    if (area->x1 > area->x2 || area->y1 > area->y2)
    {
        lv_display_flush_ready(d);
        return;
    }

    if (rt_sem_take(&flush_sem, rt_tick_from_millisecond(FLUSH_TIMEOUT_MS)) != RT_EOK)
    {
        /* Never let a stuck transfer block the UI forever. */
        lv_display_flush_ready(d);
        return;
    }

    window.x0 = area->x1;
    window.y0 = area->y1;
    window.x1 = area->x2;
    window.y1 = area->y2;

    /* These are the display coordinates, not a 0-based source rectangle: the
     * driver uses them as the layer's data area and, when they reach the last
     * panel line, to power the display on. A 0-based rectangle happens to be
     * the whole screen in a full refresh and is always the top strip in a
     * partial one, so the panel would never be turned on. */
    flush_disp = d;
    rt_device_set_tx_complete(lcd_dev, lcd_flush_done);
    /* A partial refresh consists of several transfers. Synchronize its first
     * transfer with TE, rather than waiting one panel frame for every strip. */
    wait_te = frame_start ? 1 : 0;
    rt_device_control(lcd_dev, RTGRAPHIC_CTRL_SET_NEXT_TE, &wait_te);
    frame_start = lv_display_flush_is_last(d);
    rt_graphix_ops(lcd_dev)->set_window(window.x0, window.y0, window.x1, window.y1);
    rt_graphix_ops(lcd_dev)->draw_rect_async((const char *)px_map,
                                             window.x0, window.y0,
                                             window.x1, window.y1);
}

static void lcd_wait_flush(lv_display_t *d)
{
    LV_UNUSED(d);

    if (rt_sem_take(&flush_sem, rt_tick_from_millisecond(FLUSH_TIMEOUT_MS)) == RT_EOK)
    {
        rt_sem_release(&flush_sem);
    }
    else
    {
        RT_ASSERT(0);
    }
}

static void rounder_cb(lv_event_t *e)
{
    lv_area_t *area = lv_event_get_param(e);
    uint32_t align = (lcd_info.draw_align != 0) ? lcd_info.draw_align : 1;

    area->x1 = RT_ALIGN_DOWN(area->x1, align);
    area->x2 = RT_ALIGN(area->x2 + 1, align) - 1;
    area->y1 = RT_ALIGN_DOWN(area->y1, align);
    area->y2 = RT_ALIGN(area->y2 + 1, align) - 1;
}

void lv_port_disp_init(void)
{
    uint16_t pixel_format;

    lcd_dev = rt_device_find("lcd");
    RT_ASSERT(lcd_dev != RT_NULL);

    if (rt_device_open(lcd_dev, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        RT_ASSERT(0);
        return;
    }
    rt_device_control(lcd_dev, RTGRAPHIC_CTRL_GET_INFO, &lcd_info);
    RT_ASSERT(lcd_info.width <= LCD_HOR_RES_MAX);

    switch (lv_color_format_get_size(LV_COLOR_FORMAT_DEFAULT))
    {
        case 2:
            pixel_format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
            break;
        case 3:
            pixel_format = RTGRAPHIC_PIXEL_FORMAT_RGB888;
            break;
        default:
            pixel_format = RTGRAPHIC_PIXEL_FORMAT_ARGB888;
            break;
    }
    rt_device_control(lcd_dev, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &pixel_format);

    rt_sem_init(&flush_sem, "lv_flush", 1, RT_IPC_FLAG_FIFO);

    disp = lv_display_create(lcd_info.width, lcd_info.height);
    RT_ASSERT(disp != NULL);

    lv_display_set_buffers(disp, draw_buf_1,
#if defined(LV_EXT_FB_TWO_PARTIAL)
                           draw_buf_2,
#else
                           NULL,
#endif
                           sizeof(draw_buf_1), LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_display_set_flush_cb(disp, lcd_flush);
    lv_display_set_flush_wait_cb(disp, lcd_wait_flush);
    lv_display_add_event_cb(disp, rounder_cb, LV_EVENT_INVALIDATE_AREA, NULL);
    lv_display_set_driver_data(disp, lcd_dev);
}
