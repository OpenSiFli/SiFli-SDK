/**
  ******************************************************************************
  * @file   main.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>
#include "mem_section.h"
#include "drv_epic.h"




#define BUF_WIDTH  LCD_HOR_RES_MAX
#define BUF_HEIGHT (LCD_VER_RES_MAX/16)

#define BUF_COLOR_FORMAT RTGRAPHIC_PIXEL_FORMAT_RGB888
#define BUF_PIXEL_BYTES  3

#define BUF_TOTAL_BYTES (BUF_WIDTH * BUF_HEIGHT *BUF_PIXEL_BYTES)


#if defined(SF32LB56X)  && defined(HPSYS_RAM0_SIZE)
    #define SLOW_SPEED_SRAM HPSYS_RAM0_SIZE
#else
    #define SLOW_SPEED_SRAM 0
#endif /*SF32LB56X*/

ALIGN(64) static uint8_t render_buffer[SLOW_SPEED_SRAM + BUF_TOTAL_BYTES * 2];

#define TEST_IMAGE_COLOR_FORMAT EPIC_INPUT_ARGB8565
#define TEST_IMAGE_PIXEL_BYTES 3
#define TEST_IMAGE_WIDTH  (LCD_HOR_RES_MAX*8/10)
#define TEST_IMAGE_HEIGHT (LCD_VER_RES_MAX*8/10)
L2_NON_RET_BSS_SECT_BEGIN(test_images)
L2_NON_RET_BSS_SECT(test_images, ALIGN(64) static uint8_t test_image[TEST_IMAGE_WIDTH * TEST_IMAGE_HEIGHT * TEST_IMAGE_PIXEL_BYTES]);
L2_NON_RET_BSS_SECT_END

/* Test data. */
ALIGN(4)  /* Source and destination address must be 4bytes aligned. */
const static uint8_t ezip_data_argb565[] =
{
#include "../assets/clock_simple_bg_565A.dat"
};

void dummy_func(void)
{
}



#include "drv_lcd.h"
static struct rt_semaphore lcd_sema;
static struct rt_semaphore render_done_sema;

static rt_err_t lcd_flush_done(rt_device_t dev, void *buffer)
{
    rt_sem_release(&lcd_sema);
    return RT_EOK;
}

static rt_device_t open_lcd(uint8_t *pixel_align)
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
            uint16_t framebuffer_color_format = BUF_COLOR_FORMAT;

            if (rt_device_control(lcd_device, RTGRAPHIC_CTRL_GET_INFO, &info) == RT_EOK)
            {
                *pixel_align = info.draw_align;
                rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &framebuffer_color_format);

                rt_sem_init(&lcd_sema, "rd_lcd", 1, RT_IPC_FLAG_FIFO);
                rt_device_set_tx_complete(lcd_device, lcd_flush_done);
                return lcd_device;
            }
        }
    }


    *pixel_align = 1;
    return NULL;
}


static void lcd_flush(rt_device_t dev, const EPIC_LayerConfigTypeDef *dst)
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

    err = rt_device_control(dev, SF_GRAPHIC_CTRL_LCDC_FLUSH, &flush_info);

    if (RT_EOK != err)  rt_kprintf("lcd_flush err=%d", err);
}

static void wait_lcd_flush_done(void)
{
    rt_err_t err;
    err = rt_sem_take(&lcd_sema, rt_tick_from_millisecond(1000));
    RT_ASSERT(RT_EOK == err);

    err = rt_sem_release(&lcd_sema);
    RT_ASSERT(RT_EOK == err);
}

static void partial_done_cb(drv_epic_render_list_t rl, EPIC_LayerConfigTypeDef *p_dst, void *usr_data, uint32_t last)
{
    static uint8_t lcd_te = 1;
    rt_device_t p_lcd_device = usr_data;
    rt_err_t err;
    err = rt_sem_take(&lcd_sema, rt_tick_from_millisecond(1000));
    RT_ASSERT(RT_EOK == err);
    if (last)
    {
        lcd_te = 1;
        rt_sem_release(&render_done_sema);
    }
    else
    {
        lcd_te = 0;
    }
    rt_device_control(p_lcd_device, RTGRAPHIC_CTRL_SET_NEXT_TE, &lcd_te);
    lcd_flush(p_lcd_device, p_dst);
}

static void fill_done_cb(drv_epic_render_list_t rl, EPIC_LayerConfigTypeDef *p_dst, void *usr_data, uint32_t last)
{
    if (last)
    {
        rt_sem_release(&render_done_sema);
    }
}

static void draw_fill(drv_epic_render_buf *p_buf)
{
    drv_epic_operation *o = drv_epic_alloc_op(p_buf);
    RT_ASSERT(o != NULL);


    o->op = DRV_EPIC_DRAW_FILL;
    o->clip_area.x0 = 0;
    o->clip_area.y0 = 0;
    o->clip_area.x1 = LCD_HOR_RES_MAX - 1;
    o->clip_area.y1 = LCD_VER_RES_MAX - 1;

    HAL_EPIC_LayerConfigInit(&o->mask);

    o->desc.fill.r = 64;
    o->desc.fill.g = 64;
    o->desc.fill.b = 64;
    o->desc.fill.opa = 255;

    drv_epic_commit_op(o);
}


static void draw_img(drv_epic_render_buf *p_buf)
{
    drv_epic_operation *o = drv_epic_alloc_op(p_buf);
    RT_ASSERT(o != NULL);


    o->op = DRV_EPIC_DRAW_IMAGE;
    o->clip_area.x0 = 0;
    o->clip_area.y0 = 0;
    o->clip_area.x1 = LCD_HOR_RES_MAX - 1;
    o->clip_area.y1 = LCD_VER_RES_MAX - 1;

    HAL_EPIC_LayerConfigInit(&o->mask);
    o->desc.blend.use_dest_as_bg = EPIC_BLEND_MODE_NORMAL;

    EPIC_LayerConfigTypeDef *p_src_layer = &o->desc.blend.layer;
    HAL_EPIC_LayerConfigInit(p_src_layer);
    p_src_layer->alpha = 255;
    p_src_layer->x_offset = 0;
    p_src_layer->y_offset = 0;

    p_src_layer->data = (uint8_t *)&test_image[0];
    p_src_layer->color_mode = TEST_IMAGE_COLOR_FORMAT;
    p_src_layer->width = TEST_IMAGE_WIDTH;
    p_src_layer->total_width = TEST_IMAGE_WIDTH;
    p_src_layer->height = TEST_IMAGE_HEIGHT;

    drv_epic_commit_op(o);
}

static void draw_ezip_img(drv_epic_render_buf *p_buf)
{
    drv_epic_operation *o = drv_epic_alloc_op(p_buf);
    RT_ASSERT(o != NULL);


    o->op = DRV_EPIC_DRAW_IMAGE;
    o->clip_area.x0 = 0;
    o->clip_area.y0 = 0;
    o->clip_area.x1 = LCD_HOR_RES_MAX - 1;
    o->clip_area.y1 = LCD_VER_RES_MAX - 1;

    HAL_EPIC_LayerConfigInit(&o->mask);
    o->desc.blend.use_dest_as_bg = EPIC_BLEND_MODE_NORMAL;

    EPIC_LayerConfigTypeDef *p_src_layer = &o->desc.blend.layer;
    HAL_EPIC_LayerConfigInit(p_src_layer);
    p_src_layer->alpha = 50;
    p_src_layer->x_offset = 0;
    p_src_layer->y_offset = 0;

    p_src_layer->data = (uint8_t *)&ezip_data_argb565[0];
    p_src_layer->color_mode = EPIC_INPUT_EZIP;
    p_src_layer->width = 454;
    p_src_layer->total_width = 454;
    p_src_layer->height = 454;

    drv_epic_commit_op(o);
}


static void draw_rects(drv_epic_render_buf *p_buf)
{
    int16_t w = LCD_HOR_RES_MAX - 30;
    int16_t h = 50 * 2;

    for (uint16_t y = 10; y < LCD_VER_RES_MAX - h - 1; y += (h + 20))
    {
        drv_epic_operation *o = drv_epic_alloc_op(p_buf);
        RT_ASSERT(o != NULL);


        o->op = DRV_EPIC_DRAW_RECT;
        o->clip_area.x0 = 5;
        o->clip_area.y0 = y;
        o->clip_area.x1 = o->clip_area.x0 + w - 1;
        o->clip_area.y1 = y + h - 1;

        HAL_EPIC_LayerConfigInit(&o->mask);


        o->desc.rectangle.area = o->clip_area;


        o->desc.rectangle.radius = 20;
        o->desc.rectangle.top_fillet = 1;
        o->desc.rectangle.bot_fillet = 1;
        o->desc.rectangle.argb8888 = 0xFF808080;


        drv_epic_commit_op(o);
    }
}

static void draw_borders(drv_epic_render_buf *p_buf)
{
    int16_t w = LCD_HOR_RES_MAX - 30;
    int16_t h = 50 * 2;

    for (uint16_t y = 10; y < LCD_VER_RES_MAX - h - 1; y += (h + 20))
    {
        drv_epic_operation *o = drv_epic_alloc_op(p_buf);
        RT_ASSERT(o != NULL);
        o->op = DRV_EPIC_DRAW_BORDER;
        o->clip_area.x0 = 5;
        o->clip_area.y0 = y;
        o->clip_area.x1 = o->clip_area.x0 + w - 1;
        o->clip_area.y1 = y + h - 1;

        HAL_EPIC_LayerConfigInit(&o->mask);
        o->desc.border.area = o->clip_area;

        o->desc.border.radius = 20;
        o->desc.border.width = 20;
        o->desc.border.top_side = 1;
        o->desc.border.bot_side = 1;
        o->desc.border.left_side = 1;
        o->desc.border.right_side = 1;
        o->desc.border.argb8888 = 0xFFff4040;

        drv_epic_commit_op(o);
    }
}

static void draw_arcs(drv_epic_render_buf *p_buf)
{
    for (uint16_t r = 200; r > 0; r = r - 50)
    {
        drv_epic_operation *o = drv_epic_alloc_op(p_buf);
        RT_ASSERT(o != NULL);


        o->op = DRV_EPIC_DRAW_ARC;

        HAL_EPIC_LayerConfigInit(&o->mask);

        o->desc.arc.center_x = LCD_HOR_RES_MAX >> 1;
        o->desc.arc.center_y = LCD_VER_RES_MAX >> 1;
        o->desc.arc.start_angle = 20;
        o->desc.arc.end_angle = 160;
        o->desc.arc.width = 20;

        o->desc.arc.round_start = 1;
        o->desc.arc.round_end = 1;

        o->desc.arc.radius = r;
        o->desc.arc.argb8888 = 0xFF800000;

        o->clip_area.x0 = o->desc.arc.center_x - r - 1;
        o->clip_area.y0 = o->desc.arc.center_y - r - 1;
        o->clip_area.x1 = o->desc.arc.center_x + r + 1;
        o->clip_area.y1 = o->desc.arc.center_y + r + 1;

        drv_epic_commit_op(o);
    }
}


static void draw_lines(drv_epic_render_buf *p_buf)
{
    //Horizontal line
    {
        drv_epic_operation *o = drv_epic_alloc_op(p_buf);
        RT_ASSERT(o != NULL);

        o->op = DRV_EPIC_DRAW_LINE;
        HAL_EPIC_LayerConfigInit(&o->mask);

        o->desc.line.p1.x = 20;
        o->desc.line.p1.y = LCD_VER_RES_MAX >> 1;
        o->desc.line.p2.x = LCD_HOR_RES_MAX - 20;
        o->desc.line.p2.y = LCD_VER_RES_MAX >> 1;
        o->desc.line.width = 20;

        o->desc.line.dash_width = 0;
        o->desc.line.dash_gap = 0;

        o->desc.line.round_start = 1;
        o->desc.line.round_end = 1;
        o->desc.line.raw_end = 0;
        o->desc.line.argb8888 = 0xFF00FF00;

        int16_t  half_w = o->desc.line.width >> 1;
        o->clip_area.x0 = o->desc.line.p1.x - half_w - 1;
        o->clip_area.y0 = o->desc.line.p1.y - half_w - 1;
        o->clip_area.x1 = o->desc.line.p2.x + half_w + 1;
        o->clip_area.y1 = o->desc.line.p2.y + half_w + 1;
        drv_epic_commit_op(o);
    }

    //Vertical line
    {
        drv_epic_operation *o = drv_epic_alloc_op(p_buf);
        RT_ASSERT(o != NULL);


        o->op = DRV_EPIC_DRAW_LINE;
        HAL_EPIC_LayerConfigInit(&o->mask);

        o->desc.line.p1.x = LCD_HOR_RES_MAX >> 1;
        o->desc.line.p1.y = 20;
        o->desc.line.p2.x = LCD_HOR_RES_MAX >> 1;
        o->desc.line.p2.y = LCD_VER_RES_MAX - 30;
        o->desc.line.width = 20;


        o->desc.line.dash_width = 0;
        o->desc.line.dash_gap = 0;

        o->desc.line.round_start = 1;
        o->desc.line.round_end = 1;
        o->desc.line.raw_end = 0;
        o->desc.line.argb8888 = 0xFF0000FF;


        int16_t  half_w = o->desc.line.width >> 1;
        o->clip_area.x0 = o->desc.line.p1.x - half_w - 1;
        o->clip_area.y0 = o->desc.line.p1.y - half_w - 1;
        o->clip_area.x1 = o->desc.line.p2.x + half_w + 1;
        o->clip_area.y1 = o->desc.line.p2.y + half_w + 1;
        drv_epic_commit_op(o);
    }



    //Skew line
    {
        drv_epic_operation *o = drv_epic_alloc_op(p_buf);
        RT_ASSERT(o != NULL);


        o->op = DRV_EPIC_DRAW_LINE;
        HAL_EPIC_LayerConfigInit(&o->mask);

        o->desc.line.p1.x = 40;
        o->desc.line.p1.y = 40;
        o->desc.line.p2.x = LCD_HOR_RES_MAX - 40;
        o->desc.line.p2.y = LCD_VER_RES_MAX - 40;
        o->desc.line.width = 20;


        o->desc.line.dash_width = 0;
        o->desc.line.dash_gap = 0;

        o->desc.line.round_start = 1;
        o->desc.line.round_end = 1;
        o->desc.line.raw_end = 0;
        o->desc.line.argb8888 = 0xFFFFFF00;


        int16_t  half_w = o->desc.line.width >> 1;
        o->clip_area.x0 = o->desc.line.p1.x - half_w - 1;
        o->clip_area.y0 = o->desc.line.p1.y - half_w - 1;
        o->clip_area.x1 = o->desc.line.p2.x + half_w + 1;
        o->clip_area.y1 = o->desc.line.p2.y + half_w + 1;
        drv_epic_commit_op(o);
    }

}


static void draw_letters(drv_epic_render_buf *p_buf)
{

    drv_epic_operation *o = drv_epic_alloc_op(p_buf);
    RT_ASSERT(o != NULL);
    o->op = DRV_EPIC_DRAW_LETTERS;

    HAL_EPIC_LayerConfigInit(&o->mask);
    int16_t letter_w = 30;
    int16_t letter_h = 30;
    int16_t letter_gap = 5;
    int16_t line_gap = 10;

    for (uint16_t x = 0; x < LCD_HOR_RES_MAX; x += (letter_w + letter_gap))
        for (uint16_t y = 0; y < LCD_VER_RES_MAX; y += (letter_h + line_gap))
        {
            drv_epic_letter_type_t *p_letter = drv_epic_op_alloc_letter(o);

            p_letter->data = ((uint8_t *)&test_image[0]) + (y * LCD_HOR_RES_MAX) + x;
            p_letter->area.x0 = x;
            p_letter->area.y0 = y;
            p_letter->area.x1 = x + letter_w - 1;
            p_letter->area.y1 = y + letter_h - 1;
        }

    o->desc.label.color_mode = EPIC_INPUT_A8;
    o->desc.label.r = 255;
    o->desc.label.g = 255;
    o->desc.label.b = 255;
    o->desc.label.opa = 128;

    o->clip_area.x0 = 0;
    o->clip_area.y0 = 0;
    o->clip_area.x1 = LCD_HOR_RES_MAX - 1;
    o->clip_area.y1 = LCD_VER_RES_MAX - 1;
    drv_epic_commit_op(o);
}

static void draw_polygon(drv_epic_render_buf *p_buf)
{

    drv_epic_operation *o = drv_epic_alloc_op(p_buf);
    RT_ASSERT(o != NULL);
    o->op = DRV_EPIC_DRAW_POLYGON;

    //Draw a hexagon
    HAL_EPIC_LayerConfigInit(&o->mask);
    o->desc.polygon.point_cnt = 6;
    o->desc.polygon.points[0].x = 100;
    o->desc.polygon.points[0].y = 100;
    o->desc.polygon.points[1].x = 200;
    o->desc.polygon.points[1].y = 100;
    o->desc.polygon.points[2].x = 250;
    o->desc.polygon.points[2].y = 150;
    o->desc.polygon.points[3].x = 200;
    o->desc.polygon.points[3].y = 200;
    o->desc.polygon.points[4].x = 100;
    o->desc.polygon.points[4].y = 200;
    o->desc.polygon.points[5].x = 50;
    o->desc.polygon.points[5].y = 150;


    o->desc.polygon.argb8888 = 0xFF00FF00;

    //Get the bounding box of the polygon
    for (uint16_t i = 0; i < o->desc.polygon.point_cnt; i++)
    {
        if (i == 0)
        {
            o->clip_area.x0 = o->desc.polygon.points[i].x;
            o->clip_area.y0 = o->desc.polygon.points[i].y;
            o->clip_area.x1 = o->desc.polygon.points[i].x;
            o->clip_area.y1 = o->desc.polygon.points[i].y;
        }
        else
        {
            if (o->desc.polygon.points[i].x < o->clip_area.x0) o->clip_area.x0 = o->desc.polygon.points[i].x;
            if (o->desc.polygon.points[i].y < o->clip_area.y0) o->clip_area.y0 = o->desc.polygon.points[i].y;
            if (o->desc.polygon.points[i].x > o->clip_area.x1) o->clip_area.x1 = o->desc.polygon.points[i].x;
            if (o->desc.polygon.points[i].y > o->clip_area.y1) o->clip_area.y1 = o->desc.polygon.points[i].y;
        }
    }

    //Commit the operation
    drv_epic_commit_op(o);
}

static void draw_img_3d_rotated(drv_epic_render_buf *p_buf)
{
    static int16_t angle_v = 0;
    drv_epic_operation *o = drv_epic_alloc_op(p_buf);
    RT_ASSERT(o != NULL);

    o->op = DRV_EPIC_DRAW_IMAGE;
    o->clip_area.x0 = 0;
    o->clip_area.y0 = 0;
    o->clip_area.x1 = LCD_HOR_RES_MAX - 1;
    o->clip_area.y1 = LCD_VER_RES_MAX - 1;

    HAL_EPIC_LayerConfigInit(&o->mask);
    o->desc.blend.use_dest_as_bg = EPIC_BLEND_MODE_NORMAL;

    EPIC_LayerConfigTypeDef *p_src_layer = &o->desc.blend.layer;
    HAL_EPIC_LayerConfigInit(p_src_layer);
    p_src_layer->alpha = 255;
    p_src_layer->data = (uint8_t *)&test_image[0];
    p_src_layer->color_mode = TEST_IMAGE_COLOR_FORMAT;
    p_src_layer->width = TEST_IMAGE_WIDTH;
    p_src_layer->total_width = TEST_IMAGE_WIDTH;
    p_src_layer->height = TEST_IMAGE_HEIGHT;
    //Align to the center of screen
    p_src_layer->x_offset = (LCD_HOR_RES_MAX - p_src_layer->width) >> 1;
    p_src_layer->y_offset = (LCD_VER_RES_MAX - p_src_layer->height) >> 1;

    if (angle_v < 3600)
    {
        p_src_layer->transform_cfg.type = 1;
        angle_v += 2;

        //Skip invisble angles
        if ((angle_v > 900) && (angle_v < 2700))  angle_v = 2700;
    }
    else
    {
        p_src_layer->transform_cfg.type = 2;
        angle_v += 10;
    }

    p_src_layer->transform_cfg.scale_x = EPIC_INPUT_SCALE_NONE;
    p_src_layer->transform_cfg.scale_y = EPIC_INPUT_SCALE_NONE;
    p_src_layer->transform_cfg.pivot_x = p_src_layer->x_offset + (p_src_layer->width >> 1);
    p_src_layer->transform_cfg.pivot_y = p_src_layer->y_offset + (p_src_layer->height >> 1);
    p_src_layer->transform_cfg.pivot_z = 0 - (LCD_VER_RES_MAX * 7 / 4);
    p_src_layer->transform_cfg.z_offset = 0 - (LCD_VER_RES_MAX * 5 / 4);
    p_src_layer->transform_cfg.angle_adv = angle_v;

    p_src_layer->transform_cfg.vp_x_offset = p_src_layer->transform_cfg.pivot_x;
    p_src_layer->transform_cfg.vp_y_offset = p_src_layer->transform_cfg.pivot_y;
    p_src_layer->transform_cfg.dst_z_offset = 0 - ((LCD_VER_RES_MAX * 5 / 4) * 1 /* Scale up 1*/);

    angle_v = (angle_v) % 7200;

    drv_epic_commit_op(o);
}


static void draw_img_3d_rotated_2(drv_epic_render_buf *p_buf)
{
    static int16_t angle_v = 0;

    drv_epic_operation *o = drv_epic_alloc_op(p_buf);
    RT_ASSERT(o != NULL);

    o->op = DRV_EPIC_DRAW_IMAGE;
    o->clip_area.x0 = 0;
    o->clip_area.y0 = 0;
    o->clip_area.x1 = LCD_HOR_RES_MAX - 1;
    o->clip_area.y1 = LCD_VER_RES_MAX - 1;

    HAL_EPIC_LayerConfigInit(&o->mask);
    o->desc.blend.use_dest_as_bg = EPIC_BLEND_MODE_NORMAL;

    EPIC_LayerConfigTypeDef *p_src_layer = &o->desc.blend.layer;
    HAL_EPIC_LayerConfigInit(p_src_layer);
    p_src_layer->alpha = 255;
    p_src_layer->data = (uint8_t *)&test_image[0];
    p_src_layer->color_mode = TEST_IMAGE_COLOR_FORMAT;
    p_src_layer->width = TEST_IMAGE_WIDTH >> 1;
    p_src_layer->total_width = TEST_IMAGE_WIDTH;
    p_src_layer->height = TEST_IMAGE_HEIGHT >> 1;
    //Align to the center of screen
    p_src_layer->x_offset = (LCD_HOR_RES_MAX - p_src_layer->width) >> 1;
    p_src_layer->y_offset = (LCD_VER_RES_MAX - p_src_layer->height) >> 1;


    p_src_layer->transform_cfg.type = 2;
    p_src_layer->transform_cfg.scale_x = EPIC_INPUT_SCALE_NONE;
    p_src_layer->transform_cfg.scale_y = EPIC_INPUT_SCALE_NONE;
    p_src_layer->transform_cfg.pivot_x = p_src_layer->x_offset + (p_src_layer->width >> 1);
    p_src_layer->transform_cfg.pivot_y = p_src_layer->y_offset + (p_src_layer->height >> 1);
    p_src_layer->transform_cfg.pivot_z = 0 - (LCD_VER_RES_MAX * 7 / 4);
    p_src_layer->transform_cfg.z_offset = 0 - (LCD_VER_RES_MAX * 5 / 4);
    p_src_layer->transform_cfg.angle_adv = angle_v;

    p_src_layer->transform_cfg.vp_x_offset = p_src_layer->transform_cfg.pivot_x;
    p_src_layer->transform_cfg.vp_y_offset = p_src_layer->transform_cfg.pivot_y + 300;
    p_src_layer->transform_cfg.dst_z_offset = 0 - (LCD_VER_RES_MAX * 3 / 4);

    angle_v = (angle_v + 10) % 3600;

    drv_epic_commit_op(o);
}

static void draw_arc_anim(drv_epic_render_buf *p_buf)
{
    static uint16_t end = 180;
    for (uint16_t r = 300; r > 0; r = r - 50)
    {
        drv_epic_operation *o;

        //Draw bg
        o = drv_epic_alloc_op(p_buf);
        RT_ASSERT(o != NULL);
        o->op = DRV_EPIC_DRAW_ARC;
        HAL_EPIC_LayerConfigInit(&o->mask);
        o->desc.arc.center_x = LCD_HOR_RES_MAX >> 1;
        o->desc.arc.center_y = LCD_VER_RES_MAX >> 1;
        o->desc.arc.start_angle = 180;
        o->desc.arc.end_angle = 360;
        o->desc.arc.width = 20;
        o->desc.arc.round_start = 0;
        o->desc.arc.round_end = 0;
        o->desc.arc.radius = r;
        o->desc.arc.argb8888 = 0xFF440000;
        o->clip_area.x0 = o->desc.arc.center_x - r - 1;
        o->clip_area.y0 = o->desc.arc.center_y - r - 1;
        o->clip_area.x1 = o->desc.arc.center_x + r + 1;
        o->clip_area.y1 = o->desc.arc.center_y + r + 1;
        drv_epic_commit_op(o);


        //Draw fg
        o = drv_epic_alloc_op(p_buf);
        RT_ASSERT(o != NULL);
        o->op = DRV_EPIC_DRAW_ARC;
        HAL_EPIC_LayerConfigInit(&o->mask);
        o->desc.arc.center_x = LCD_HOR_RES_MAX >> 1;
        o->desc.arc.center_y = LCD_VER_RES_MAX >> 1;
        o->desc.arc.start_angle = 180;
        o->desc.arc.end_angle = end + 1;
        end = 180 + ((end + 1) % 180);
        o->desc.arc.width = 20;
        o->desc.arc.round_start = 0;
        o->desc.arc.round_end = 0;
        o->desc.arc.radius = r;
        o->desc.arc.argb8888 = 0xFFff0000;
        o->clip_area.x0 = o->desc.arc.center_x - r - 1;
        o->clip_area.y0 = o->desc.arc.center_y - r - 1;
        o->clip_area.x1 = o->desc.arc.center_x + r + 1;
        o->clip_area.y1 = o->desc.arc.center_y + r + 1;
        drv_epic_commit_op(o);
    }
}

static void generate_image(unsigned char *buffer, uint32_t cf, int HOR_MAX, int VER_MAX)
{
    srand(time(NULL));
    int remaining_area = LCD_HOR_RES_MAX * LCD_VER_RES_MAX;
#define  grid_count  16
#define MIN_SIZE  20

    //1. Calculate the fill areas
    EPIC_AreaTypeDef areas[grid_count];

    for (int i = 0; i < grid_count; i++)
    {
        int max_area = remaining_area / (grid_count - i);
        if (max_area < MIN_SIZE * MIN_SIZE)
        {
            max_area = MIN_SIZE * MIN_SIZE;
        }
        int area = rand() % (max_area - MIN_SIZE * MIN_SIZE + 1) + MIN_SIZE * MIN_SIZE;

        // random width&height
        int width = MIN_SIZE;
        int height = area / width;
        if (area > MIN_SIZE * MIN_SIZE)
        {
            width = rand() % (area / MIN_SIZE - MIN_SIZE + 1) + MIN_SIZE;
            height = area / width;
            if (height < MIN_SIZE)
            {
                height = MIN_SIZE;
                width = area / height;
            }
        }

        // random start x,y
        int start_x = rand() % (LCD_HOR_RES_MAX - width + 1);
        int start_y = rand() % (LCD_VER_RES_MAX - height + 1);

        //Save area and color
        areas[i].x0 = start_x;
        areas[i].y0 = start_y;
        areas[i].x1 = start_x + width - 1;
        areas[i].y1 = start_y + height - 1;


        remaining_area -= area;
    }




    //2. Fill the areas
    drv_epic_render_buf render_buf;
    render_buf.cf = cf;
    render_buf.data = (uint8_t *)buffer;
    render_buf.area.x0 = 0;
    render_buf.area.y0 = 0;
    render_buf.area.x1 = LCD_HOR_RES_MAX - 1;
    render_buf.area.y1 = LCD_VER_RES_MAX - 1;

    drv_epic_render_list_t rl;
    EPIC_AreaTypeDef ow_area;
    rl = drv_epic_alloc_render_list(&render_buf, &ow_area);
    RT_ASSERT(rl != NULL);

    /*Clear render buffer*/
    drv_epic_operation *o = drv_epic_alloc_op(&render_buf);
    RT_ASSERT(o != NULL);
    o->op = DRV_EPIC_DRAW_FILL;
    o->clip_area = render_buf.area;
    o->desc.fill.r = 0;
    o->desc.fill.g = 0;
    o->desc.fill.b = 0;
    o->desc.fill.opa = 0;
    drv_epic_commit_op(o);

    for (int i = 0; i < grid_count; i++)
    {
        o = drv_epic_alloc_op(&render_buf);
        RT_ASSERT(o != NULL);

        o->op = DRV_EPIC_DRAW_RECT;
        o->clip_area.x0 = areas[i].x0;
        o->clip_area.y0 = areas[i].y0;
        o->clip_area.x1 = areas[i].x1;
        o->clip_area.y1 = areas[i].y1;

        o->desc.rectangle.area = o->clip_area;

        o->desc.rectangle.argb8888 = 0x2F000000 | rand();

        drv_epic_commit_op(o);
    }

    /*Start rendering*/
    EPIC_MsgTypeDef msg;
    msg.id = EPIC_MSG_RENDER_TO_BUF;
    msg.render_list = rl;
    msg.content.r2b.dst_area.x0 = 0;
    msg.content.r2b.dst_area.y0 = 0;
    msg.content.r2b.dst_area.x1 = HOR_MAX - 1;
    msg.content.r2b.dst_area.y1 = VER_MAX - 1;
    msg.content.r2b.usr_data = NULL;
    msg.content.r2b.done_cb = fill_done_cb;

    drv_epic_render_msg_commit(&msg);

    /*Wait rendering done.*/
    rt_err_t err;
    err = rt_sem_take(&render_done_sema, rt_tick_from_millisecond(10000));
    RT_ASSERT(RT_EOK == err);
}

int main(void)
{
    rt_kprintf("__main start\r\n");

    /*
      1 - Show the rendering result on LCD
      0 - Do not show the rendering result on LCD, just render to buffer.
    */
    uint8_t show_on_lcd = 1;
    uint8_t pixel_align;
    rt_device_t lcd_device = open_lcd(&pixel_align);
    if (!lcd_device)
    {
        rt_kprintf("Open LCD fail!\r\n");
        return 0;
    }

    drv_gpu_open();
    drv_epic_setup_render_buffer(
        (uint8_t *)&render_buffer[SLOW_SPEED_SRAM],
        (uint8_t *)&render_buffer[SLOW_SPEED_SRAM + BUF_TOTAL_BYTES],
        BUF_TOTAL_BYTES);
    rt_sem_init(&render_done_sema, "rd_done_sema", 0, RT_IPC_FLAG_FIFO);


    rt_kprintf("generate_image start\r\n");
    generate_image((uint8_t *)&test_image[0], TEST_IMAGE_COLOR_FORMAT, TEST_IMAGE_WIDTH, TEST_IMAGE_HEIGHT);
    rt_kprintf("generate_image done\r\n");

    drv_epic_render_buf virtual_render_buf;
    virtual_render_buf.cf = EPIC_INPUT_RGB888;
    virtual_render_buf.data = (uint8_t *)0xCCCCCCCC;
    virtual_render_buf.area.x0 = 0;
    virtual_render_buf.area.y0 = 0;
    virtual_render_buf.area.x1 = LCD_HOR_RES_MAX - 1;
    virtual_render_buf.area.y1 = LCD_VER_RES_MAX - 1;


    while (1)
    {
        /*Allocate new render list*/
        drv_epic_render_list_t rl;
        EPIC_AreaTypeDef ow_area;
        rl = drv_epic_alloc_render_list(&virtual_render_buf, &ow_area);
        RT_ASSERT(rl != NULL);


        /*Draw somthing*/
        draw_fill(&virtual_render_buf);
        //draw_img(&virtual_render_buf);
        draw_ezip_img(&virtual_render_buf);
        draw_rects(&virtual_render_buf);
        draw_borders(&virtual_render_buf);
        draw_arcs(&virtual_render_buf);//arc
        draw_lines(&virtual_render_buf);
        draw_letters(&virtual_render_buf);
        draw_polygon(&virtual_render_buf);

        // generate_image((uint8_t *)&test_image[0], TEST_IMAGE_COLOR_FORMAT, TEST_IMAGE_WIDTH, TEST_IMAGE_HEIGHT);
        //draw_img_3d_rotated(&virtual_render_buf);
        //draw_img_3d_rotated_2(&virtual_render_buf);
        //draw_arc_anim(&virtual_render_buf);


        /*Start rendering  and show the result on LCD*/
        EPIC_MsgTypeDef msg;
        msg.id = EPIC_MSG_RENDER_DRAW;
        msg.render_list = rl;
        msg.content.rd.area.x0 = 0;
        msg.content.rd.area.y0 = 0;
        msg.content.rd.area.x1 = LCD_HOR_RES_MAX - 1;
        msg.content.rd.area.y1 = LCD_VER_RES_MAX - 1;
        msg.content.rd.usr_data = lcd_device;
        msg.content.rd.pixel_align = pixel_align;

        if (show_on_lcd)
        {
            /*and show the resule on LCD at first time*/
            msg.content.rd.partial_done_cb = partial_done_cb;
        }
        else
        {
            msg.content.rd.partial_done_cb = NULL;
        }

        drv_epic_render_msg_commit(&msg);

        if (show_on_lcd)
        {
            /*Wait rendering done.*/
            rt_err_t err;
            err = rt_sem_take(&render_done_sema, rt_tick_from_millisecond(3000));
            RT_ASSERT(RT_EOK == err);
            /*Wait LCD asynchronize flushing done.*/
            wait_lcd_flush_done();
        }
    }


    return RT_EOK;
}




/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

