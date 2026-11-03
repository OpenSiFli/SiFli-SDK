/*
 * SPDX-FileCopyrightText: 2021-2021 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "drv_lcd.h"

#ifdef BSP_USING_LCD_FRAMEBUFFER
    #include "mem_section.h"
    #include "drv_lcd_fb.h"
    #define FB_LINE_SIZE RT_ALIGN(LCD_HOR_RES_MAX * 3, 64) //Define an maximum line size, as we don't know the actual color format
    #define FB_TYPE      uint8_t
    L2_NON_RET_BSS_SECT_BEGIN(frambuf)
    L2_NON_RET_BSS_SECT(frambuf, ALIGN(64) static FB_TYPE buf2_1[FB_LINE_SIZE * LCD_VER_RES_MAX]);
    L2_NON_RET_BSS_SECT_END

    #if defined(BSP_USING_RAMLESS_LCD)
        L2_NON_RET_BSS_SECT_BEGIN(frambuf)
        L2_NON_RET_BSS_SECT(frambuf, ALIGN(64) static FB_TYPE buf2_2[FB_LINE_SIZE * LCD_VER_RES_MAX]);
        L2_NON_RET_BSS_SECT_END
    #endif
#endif /* BSP_USING_LCD_FRAMEBUFFER */

static struct rt_semaphore lcd_sema;






#ifdef BSP_USING_LCD_FRAMEBUFFER
static void update_fb(const EPIC_LayerConfigTypeDef *dst, uint32_t last)
{
    lcd_fb_desc_t fb_dsc;
    static uint8_t switch_fb = 1;
    static FB_TYPE *current_fb = NULL;

    if (switch_fb)
    {
#if defined(BSP_USING_RAMLESS_LCD)
        current_fb = (current_fb == &buf2_1[0]) ? &buf2_2[0] : &buf2_1[0];

#else
        if (NULL == current_fb)
            current_fb = &buf2_1[0];
        else
            return; //No need to update fb if not switch
#endif /* BSP_USING_RAMLESS_LCD */

#ifdef DRV_LCD_COMPRESSED_BUF_AVALIABLE
        switch (dst->color_mode)
        {
        case EPIC_COLOR_RGB888:
            fb_dsc.format = RTGRAPHIC_PIXEL_FORMAT_RGB888;
            fb_dsc.line_bytes = LCD_HOR_RES_MAX * 3;
            fb_dsc.line_bytes = CMPR_3_RGB888_COMPRESSED_BYTES(LCD_HOR_RES_MAX);
            fb_dsc.cmpr_rate = 3;
            break;
        case EPIC_COLOR_RGB565:
            fb_dsc.format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
            fb_dsc.line_bytes = LCD_HOR_RES_MAX * 2;
            fb_dsc.line_bytes = CMPR_1_RGB565_COMPRESSED_BYTES(LCD_HOR_RES_MAX);
            fb_dsc.cmpr_rate = 1;
            break;
        default:
            RT_ASSERT(0); //Invalid color mode
            break;
        }
#else
        switch (dst->color_mode)
        {
        case EPIC_COLOR_RGB888:
            fb_dsc.format = RTGRAPHIC_PIXEL_FORMAT_RGB888;
            fb_dsc.line_bytes = LCD_HOR_RES_MAX * 3;
            fb_dsc.cmpr_rate = 0;
            break;
        case EPIC_COLOR_RGB565:
            fb_dsc.format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
            fb_dsc.line_bytes = LCD_HOR_RES_MAX * 2;
            fb_dsc.cmpr_rate = 0;
            break;
        default:
            RT_ASSERT(0); //Invalid color mode
            break;
        }
#endif
        fb_dsc.area.x0 = 0;
        fb_dsc.area.y0 = 0;
        fb_dsc.area.x1 = LCD_HOR_RES_MAX - 1;
        fb_dsc.area.y1 = LCD_VER_RES_MAX - 1;
        fb_dsc.p_data = (uint8_t *)current_fb;


        drv_lcd_fb_set(&fb_dsc);
    }

    switch_fb = last;
}

static void lcd_flush_done(lcd_fb_desc_t *fb_desc)
{
    rt_sem_release(&lcd_sema);
}
void lcd_flush(rt_device_t dev, const EPIC_LayerConfigTypeDef *dst, uint32_t last)
{
    rt_err_t err;
    LCD_AreaDef src_area =
    {
        .x0 = dst->x_offset,
        .y0 = dst->y_offset,
        .x1 = dst->x_offset + dst->width - 1,
        .y1 = dst->y_offset + dst->height - 1
    };

    LCD_AreaDef clip_area = src_area;

    err = rt_sem_take(&lcd_sema, rt_tick_from_millisecond(1000));
    RT_ASSERT(RT_EOK == err);

    update_fb(dst, last);
    drv_lcd_fb_write_send(&clip_area, &src_area, (uint8_t *)dst->data, lcd_flush_done,
                          (uint8_t)last);
}
#else /* !BSP_USING_LCD_FRAMEBUFFER */
static rt_err_t lcd_flush_done(rt_device_t dev, void *buffer)
{
    rt_sem_release(&lcd_sema);
    return RT_EOK;
}

void lcd_flush(rt_device_t dev, const EPIC_LayerConfigTypeDef *dst, uint32_t last)
{
    lcd_flush_info_t flush_info;
    rt_err_t err;

    flush_info.cmpr_rate = 0;
    flush_info.pixel      = dst->data;
    switch (dst->color_mode)
    {
    case EPIC_COLOR_RGB888:
        flush_info.color_format = RTGRAPHIC_PIXEL_FORMAT_RGB888;
        break;
    case EPIC_COLOR_RGB565:
    default:
        flush_info.color_format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
        break;
    }

    flush_info.window.x0 = dst->x_offset;
    flush_info.window.x1 = dst->x_offset + dst->width - 1;
    flush_info.window.y0 = dst->y_offset;
    flush_info.window.y1 = dst->y_offset + dst->height - 1;

    flush_info.pixel_area = flush_info.window;

    err = rt_sem_take(&lcd_sema, rt_tick_from_millisecond(1000));
    RT_ASSERT(RT_EOK == err);

    uint8_t lcd_te = last;


    rt_device_control(dev, RTGRAPHIC_CTRL_SET_NEXT_TE, &lcd_te);
    rt_device_set_tx_complete(dev, lcd_flush_done);
    err = rt_device_control(dev, SF_GRAPHIC_CTRL_LCDC_FLUSH, &flush_info);

    if (RT_EOK != err)  rt_kprintf("lcd_flush err=%d", err);
}
#endif /* BSP_USING_LCD_FRAMEBUFFER */
void wait_lcd_flush_done(void)
{
    rt_err_t err;
    err = rt_sem_take(&lcd_sema, rt_tick_from_millisecond(1000));
    RT_ASSERT(RT_EOK == err);

    err = rt_sem_release(&lcd_sema);
    RT_ASSERT(RT_EOK == err);
}

rt_device_t open_lcd(uint8_t *pixel_align, uint16_t framebuffer_color_format)
{
    /*
        Open LCD Device and get LCD infomation
    */
    rt_device_t lcd_device = rt_device_find("lcd");

    if (lcd_device)
    {
        if (rt_device_open(lcd_device, RT_DEVICE_OFLAG_RDWR) == RT_EOK)
        {
            struct rt_device_graphic_info info;

            if (rt_device_control(lcd_device, RTGRAPHIC_CTRL_GET_INFO, &info) == RT_EOK)
            {
                *pixel_align = info.draw_align;
                rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &framebuffer_color_format);

                rt_sem_init(&lcd_sema, "rd_lcd", 1, RT_IPC_FLAG_FIFO);
#ifdef BSP_USING_LCD_FRAMEBUFFER
                drv_lcd_fb_init("lcd");
#endif /* BSP_USING_LCD_FRAMEBUFFER */
                return lcd_device;
            }
        }
    }

    *pixel_align = 1;
    return NULL;
}



