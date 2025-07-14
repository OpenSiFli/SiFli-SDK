/**
  ******************************************************************************
  * @file   gpu_lcd.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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
#include "lvgl.h"
#include "board.h"
#include "drv_lcd.h"
#include "log.h"
#include "bf0_pm.h"
#include "section.h"
#include "mem_section.h"
#ifdef RT_USING_DFS
    #include <dfs_posix.h>
#endif
#include "drv_epic.h"
#include "lvsf_perf.h"

#include "cpu_usage_profiler.h"


#include "lv_display_private.h"
#include "lv_refr_private.h"

#if defined(BSP_USING_LCD_FRAMEBUFFER)
    #include "drv_lcd_fb.h"
#endif

#if (16 != LV_COLOR_DEPTH) && (24 != LV_COLOR_DEPTH) && (32 != LV_COLOR_DEPTH)
    #error "Unsupported color depth"
#endif

/*LCD resolution can't small than LVGL's*/
#ifdef BSP_USING_LCD
    #if (LV_HOR_RES_MAX>LCD_HOR_RES_MAX)
        #error "LV_HOR_RES_MAX>LCD_HOR_RES_MAX, Select right LCD please!"
    #endif

    #if (LV_VER_RES_MAX>LCD_VER_RES_MAX)
        #error "LV_VER_RES_MAX>LCD_VER_RES_MAX, Select right LCD please!"
    #endif
#endif /* BSP_USING_LCD */


#if defined(LV_FB_TWO_SCREEN_SIZE) || defined(LV_FB_ONE_SCREEN_SIZE)
    #define LV_FB_LINE_NUM LV_VER_RES_MAX
#endif /* LV_FB_TWO_SCREEN_SIZE || LV_FB_ONE_SCREEN_SIZE*/
#if defined(LV_FB_ONE_SCREEN_SIZE) && defined(LCD_FB_USING_NONE)
    #error "Not supported on v9 now!"
#endif


#define LCD_FLUSH_EXP_MS   (5000)//Include LCD reset time



#ifdef FRAME_BUFFER_IN_PSRAM
    #define FRAME_BUFFER_BSS_SECT_BEGIN(frambuf) L2_NON_RET_BSS_SECT_BEGIN(frambuf)
    #define FRAME_BUFFER_BSS_SECT_END L2_NON_RET_BSS_SECT_END
    #define FRAME_BUFFER_BSS_SECT(frambuf, var) L2_NON_RET_BSS_SECT(frambuf, var)
#else
    #define FRAME_BUFFER_BSS_SECT_BEGIN(frambuf) L1_NON_RET_BSS_SECT_BEGIN(frambuf)
    #define FRAME_BUFFER_BSS_SECT_END L1_NON_RET_BSS_SECT_END
    #define FRAME_BUFFER_BSS_SECT(frambuf, var) L1_NON_RET_BSS_SECT(frambuf, var)
#endif

extern void perf_monitor(lv_display_t *disp_drv, uint32_t time, uint32_t px);

#ifdef LV_USE_LVSF
    #define debug_lcd_flush_start(x1, y1, x2, y2)  LV_DEBUG_LCD_FLUSH_START(x1, y1, x2, y2)
    #define debug_lcd_flush_end()                  LV_DEBUG_LCD_FLUSH_STOP()
#else
    #define debug_lcd_flush_start(x1, y1, x2, y2)
    #define debug_lcd_flush_end()
#endif /* LV_USE_LVSF */



static rt_device_t device;
static struct rt_device_graphic_info info;
static struct rt_semaphore lcd_sema;


static lv_display_t *lcd_flushing_disp_drv = NULL;
static lv_display_t *disp;

#if (LV_COLOR_DEPTH == 24)
    typedef lv_color_t lv_fb_color_t;
#elif (LV_COLOR_DEPTH == 16)
    typedef lv_color16_t lv_fb_color_t;
#endif /* LV_COLOR_DEPTH == 24*/


/**************************************************
   1. Defination of LVGL buffer(s) on SRAM
****************************************************/
FRAME_BUFFER_BSS_SECT_BEGIN(frambuf)
FRAME_BUFFER_BSS_SECT(frambuf, ALIGN(FB_ALIGN_BYTE) static lv_fb_color_t  buf1_1[FB_ALIGNED_HOR_RES * LV_FB_LINE_NUM]);
#if defined(LV_FB_TWO_NOT_SCREEN_SIZE) || defined(LV_FB_TWO_SCREEN_SIZE)
    FRAME_BUFFER_BSS_SECT(frambuf, ALIGN(FB_ALIGN_BYTE) static lv_fb_color_t  buf1_2[FB_ALIGNED_HOR_RES * LV_FB_LINE_NUM]);
#endif /* LV_FB_TWO_NOT_SCREEN_SIZE || LV_FB_TWO_SCREEN_SIZE */
FRAME_BUFFER_BSS_SECT_END



/**************************************************
   2. Defination of LCD buffer(s) on PSRAM
****************************************************/
#ifdef LCD_FB_USING_AUTO
    #if   defined(BSP_USING_RAMLESS_LCD) && defined(DRV_LCD_COMPRESSED_BUF_AVALIABLE)
        #define LCD_FB_USING_TWO_COMPRESSED
    #elif defined(BSP_USING_RAMLESS_LCD) && !defined(DRV_LCD_COMPRESSED_BUF_AVALIABLE)
        #define LCD_FB_USING_TWO_UNCOMPRESSED
    #elif !defined(BSP_USING_RAMLESS_LCD) && defined(DRV_LCD_COMPRESSED_BUF_AVALIABLE)
        #define LCD_FB_USING_ONE_COMPRESSED
    #elif !defined(BSP_USING_RAMLESS_LCD) && !defined(DRV_LCD_COMPRESSED_BUF_AVALIABLE)
        #define LCD_FB_USING_ONE_UNCOMPRESSED
    #endif
#endif /* LCD_FB_USING_AUTO */



#ifdef LCD_FB_USING_NONE
    #if defined(LV_FB_ONE_SCREEN_SIZE)
        #define FB_LINE_SIZE FB_ALIGNED_HOR_RES
        #define FB_TYPE      lv_fb_color_t

        #undef get_draw_buf
        #define get_draw_buf()  (&buf1_1[0])
    #endif
#else
    #if defined(LCD_FB_USING_ONE_UNCOMPRESSED) || defined(LCD_FB_USING_TWO_UNCOMPRESSED)
        #define FB_LINE_SIZE FB_ALIGNED_HOR_RES
        #define FB_TYPE      lv_fb_color_t

    #elif defined(LCD_FB_USING_ONE_COMPRESSED) || defined(LCD_FB_USING_TWO_COMPRESSED)
        #define FB_TYPE      uint32_t
        #if (LV_COLOR_DEPTH == 24)
            #define FB_CMPR_RATE 3
            #define FB_LINE_SIZE TARGET_SIZE_TO_CMPR_WORDS(CMPR_3_RGB888_TGT_SIZE(RT_ALIGN(LV_HOR_RES_MAX*3, 4)/4))
        #elif (LV_COLOR_DEPTH == 16)
            #define FB_CMPR_RATE 1
            #define FB_LINE_SIZE TARGET_SIZE_TO_CMPR_WORDS(CMPR_1_RGB565_TGT_SIZE(RT_ALIGN(LV_HOR_RES_MAX*2, 4)/4))
        #endif /* LV_COLOR_DEPTH == 24*/
    #endif /* LCD_FB_USING_ONE_UNCOMPRESSED */

    /* screen sized buffer on low speed RAM
    64 algined buffer is more efficient for EPIC
    */
    L2_NON_RET_BSS_SECT_BEGIN(frambuf)
    L2_NON_RET_BSS_SECT(frambuf, ALIGN(64) static FB_TYPE buf2_1[FB_LINE_SIZE * LV_VER_RES_MAX]);
    L2_NON_RET_BSS_SECT_END

    #if defined(LCD_FB_USING_TWO_UNCOMPRESSED)||defined(LCD_FB_USING_TWO_COMPRESSED)
        L2_NON_RET_BSS_SECT_BEGIN(frambuf)
        L2_NON_RET_BSS_SECT(frambuf, ALIGN(64) static FB_TYPE buf2_2[FB_LINE_SIZE * LV_VER_RES_MAX]);
        L2_NON_RET_BSS_SECT_END

        static FB_TYPE *using_buf; /*ramless LCD using fb*/
        #undef get_draw_buf
        #define get_draw_buf()  ((using_buf == &buf2_1[0]) ? &buf2_2[0] : &buf2_1[0])
        #define switch_draw_buf()  (using_buf = get_draw_buf())
    #else
        #undef get_draw_buf
        #define get_draw_buf()  (&buf2_1[0])
    #endif
#endif /* LCD_FB_USING_NONE */

static void dummy_func2(void)
{
};


static void dummy_func3(void)
{
}


#ifdef BSP_USING_LCD_FRAMEBUFFER
static void update_fb(void)
{
    lcd_fb_desc_t fb_dsc;
    uint16_t cf;

    if (16 == LV_COLOR_DEPTH)
        cf = RTGRAPHIC_PIXEL_FORMAT_RGB565;
    else if (24 == LV_COLOR_DEPTH)
        cf = RTGRAPHIC_PIXEL_FORMAT_RGB888;
    else if (32 == LV_COLOR_DEPTH)
        cf = RTGRAPHIC_PIXEL_FORMAT_ARGB888;
    else
    {
        //Fix me
        RT_ASSERT(0);
    }

    fb_dsc.area.x0 = disp->offset_x;
    fb_dsc.area.y0 = disp->offset_y;
    fb_dsc.area.x1 = disp->offset_x + disp->hor_res - 1;
    fb_dsc.area.y1 = disp->offset_y + disp->ver_res - 1;
    fb_dsc.p_data = (uint8_t *)get_draw_buf();
#ifdef FB_CMPR_RATE
    fb_dsc.cmpr_rate = FB_CMPR_RATE;
#else
    fb_dsc.cmpr_rate = 0;
#endif /* FB_CMPR_RATE */
    fb_dsc.line_bytes = FB_LINE_SIZE * sizeof(FB_TYPE);
    fb_dsc.format = cf;
    drv_lcd_fb_set(&fb_dsc);
}
#endif /* BSP_USING_LCD_FRAMEBUFFER */



/*
    Some LCD (ie. Rydium DSI LCD):
    The SC and EC-SC+1 must can be divisible by 2, (SC - Start Column, EC - End Column), and row too.

    SPD LCD need align to 4
*/
static void rounder_cb(lv_event_t *e)
{
    lv_area_t *area = lv_event_get_param(e);
    uint32_t align_size = (info.draw_align != 0) ? info.draw_align : 1;

    area->x1 = RT_ALIGN_DOWN(area->x1, align_size);
    area->x2 = RT_ALIGN(area->x2 + 1, align_size) - 1;

    area->y1 = RT_ALIGN_DOWN(area->y1, align_size);
    area->y2 = RT_ALIGN(area->y2 + 1, align_size) - 1;


#ifdef FB_CMPR_RATE
    /*Extend to whole line if FB compression is enabled.*/
    area->x1 = 0;
    area->x2 = LV_HOR_RES_MAX - 1;
#endif /* FB_CMPR_RATE */

}

static void render_start(lv_event_t *e)
{
#if defined(LCD_FB_USING_TWO_COMPRESSED)||defined(LCD_FB_USING_TWO_UNCOMPRESSED)
    if (disp->inv_p > 0)
    {
        const lv_area_t scr_area = { 0, 0, LV_HOR_RES_MAX - 1, LV_VER_RES_MAX - 1};
        lv_inv_area(disp, &scr_area); //To avoid copying PSRAM->PSRAM
    }
#endif


#if defined(LCD_FB_USING_TWO_COMPRESSED)||defined(LCD_FB_USING_TWO_UNCOMPRESSED)
    switch_draw_buf();
    update_fb();
#endif /* LCD_FB_USING_TWO_COMPRESSED ||  LCD_FB_USING_TWO_UNCOMPRESSED*/

}



static void wait_flush_done(lv_display_t *disp_drv)
{
#if 0//def BSP_USING_LCD_FRAMEBUFFER
    drv_lcd_fb_wait_write_done(LCD_FLUSH_EXP_MS);
#endif /* BSP_USING_LCD_FRAMEBUFFER */
    rt_err_t err;
    err = rt_sem_take(&lcd_sema, rt_tick_from_millisecond(LCD_FLUSH_EXP_MS));
    RT_ASSERT(RT_EOK == err);
    err = rt_sem_release(&lcd_sema);
    RT_ASSERT(RT_EOK == err);
}


#ifdef BSP_USING_LCD_FRAMEBUFFER
static void lcd_flush_done(lcd_fb_desc_t *fb_desc)
{
    lv_display_t *p_disp = lcd_flushing_disp_drv;
    lcd_flushing_disp_drv = NULL;

    rt_err_t err;
    err = rt_sem_release(&lcd_sema);
    RT_ASSERT(RT_EOK == err);

    /* Inform the graphics library that you are ready with the flushing*/
    p_disp->flushing = 0;
}
#else
static rt_err_t lcd_flush_done(rt_device_t dev, void *buffer)
{
    lv_display_t *p_disp = lcd_flushing_disp_drv;
    lcd_flushing_disp_drv = NULL;
    debug_lcd_flush_end();

    rt_err_t err;
    err = rt_sem_release(&lcd_sema);
    RT_ASSERT(RT_EOK == err);

    /* Inform the graphics library that you are ready with the flushing*/
    p_disp->flushing = 0;

    return RT_EOK;
}

#endif /* BSP_USING_LCD_FRAMEBUFFER */

uint8_t drv_gpu_is_cached_ram(uint32_t start, uint32_t len)
{
#ifndef LCD_FB_USING_NONE
    uint32_t uncached_start, uncached_end;

    uncached_start = (uint32_t)&buf2_1[0];
    uncached_end = uncached_start + sizeof(buf2_1);
    if ((start >= uncached_start) && (start + len <= uncached_end))
    {
        return 0;
    }


#if defined(switch_draw_buf)
    uncached_start = (uint32_t)&buf2_2[0];
    uncached_end = uncached_start + sizeof(buf2_2);
    if ((start >= uncached_start) && (start + len <= uncached_end))
    {
        return 0;
    }
#endif
#endif /* LCD_FB_USING_NONE */


    return 1;
}

static void lcd_flush(lv_display_t *disp_drv, const lv_area_t *refresh_area, uint8_t *color_p)
{
    const lv_area_t *p_buf_area = &disp_drv->layer_head->buf_area;

    LCD_AreaDef clip_area =   //Buf clip area
    {
        .x0 = refresh_area->x1 + disp_drv->offset_x,
        .y0 = refresh_area->y1 + disp_drv->offset_y,
        .x1 = refresh_area->x2 + disp_drv->offset_x,
        .y1 = refresh_area->y2 + disp_drv->offset_y
    };

    LCD_AreaDef src_area =
    {
        .x0 = p_buf_area->x1,
        .y0 = p_buf_area->y1,
        .x1 = p_buf_area->x2,
        .y1 = p_buf_area->y2
    };

    //BUG 1944: Invalid coordinates if framebuffer larger than LCD and refresh area out of screen
    if ((clip_area.x0 > clip_area.x1) || (clip_area.y0 > clip_area.y1))
    {
        /* Inform the graphics library that you are ready with the flushing*/
        lv_disp_flush_ready(disp_drv);
        return;
    }






    rt_err_t err;
    err = rt_sem_take(&lcd_sema, rt_tick_from_millisecond(LCD_FLUSH_EXP_MS));
    RT_ASSERT(RT_EOK == err);

    lcd_flushing_disp_drv = disp_drv;

#ifdef BSP_USING_LCD_FRAMEBUFFER
    drv_lcd_fb_write_send(&clip_area, &src_area, (uint8_t *)color_p, lcd_flush_done,
                          disp_drv->flushing_last);
#else
    debug_lcd_flush_start(clip_area.x0, clip_area.y0, clip_area.x1, clip_area.y1);
    rt_device_set_tx_complete(device, lcd_flush_done);
    rt_graphix_ops(device)->set_window(clip_area.x0, clip_area.y0, clip_area.x1, clip_area.y1);
    rt_graphix_ops(device)->draw_rect_async((const char *)color_p, src_area.x0,
                                            src_area.y0, src_area.x1, src_area.y1);
#endif /* BSP_USING_LCD_FRAMEBUFFER */

}
void *get_disp_buf(uint32_t size)
{
    lv_display_t *disp = lv_display_get_default();
    if (!disp)
    {
        LOG_E("Display not initialized");
        return NULL;
    }

    lv_draw_buf_t *draw_buf = lv_display_get_buf_active(disp);
    if (!draw_buf || !draw_buf->data)
    {
        LOG_E("Active draw buffer is NULL or invalid");
        return NULL;
    }

    uint32_t buf_size = draw_buf->header.stride * draw_buf->header.h * lv_color_format_get_size(draw_buf->header.cf);
    if (buf_size >= size)
    {
        return (void *)draw_buf->data;
    }

#if defined(LV_FB_TWO_NOT_SCREEN_SIZE) || defined(LV_FB_TWO_SCREEN_SIZE)
    if (disp->buf_1 && disp->buf_2 &&
            draw_buf == disp->buf_1 &&
            (uint32_t)((uintptr_t)disp->buf_2->data - (uintptr_t)disp->buf_1->data) == buf_size &&
            2 * buf_size >= size)
    {
        return (void *)disp->buf_1->data;
    }
#endif

#ifndef LCD_FB_USING_NONE
    if (size <= sizeof(buf2_1))
    {
        return (void *)get_draw_buf();
    }
#endif

    LOG_W("No suitable display buffer found for size: %u", size);
    return NULL;
}






void lv_lcd_init(const char *name)
{
    uint16_t cf;
    uint32_t i;


    /* LCD Device Init */
    device = rt_device_find(name);
    RT_ASSERT(device != RT_NULL);

    if (rt_device_open(device, RT_DEVICE_OFLAG_RDWR) == RT_EOK)
    {
        rt_device_control(device, RTGRAPHIC_CTRL_GET_INFO, &info);
    }


    if ((info.bits_per_pixel != LV_COLOR_DEPTH) && (info.bits_per_pixel != 32 && LV_COLOR_DEPTH != 24))
    {

        rt_kprintf("Warning: framebuffer color depth(%d) mismatch! (Should match with LV_COLOR_DEPTH)\n",
                   info.bits_per_pixel);

    }
    /*
        Set framebuffer color format
    */
    if (16 == LV_COLOR_DEPTH)
        cf = RTGRAPHIC_PIXEL_FORMAT_RGB565;
    else if (24 == LV_COLOR_DEPTH)
        cf = RTGRAPHIC_PIXEL_FORMAT_RGB888;
    else if (32 == LV_COLOR_DEPTH)
        cf = RTGRAPHIC_PIXEL_FORMAT_ARGB888;
    else
    {
        //Fix me
        RT_ASSERT(0);
    }

    rt_device_control(device, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &cf);



    disp = lv_display_create(LV_HOR_RES_MAX, LV_VER_RES_MAX);
    if (disp == NULL)
    {
        RT_ASSERT(0);
    }


    static lv_draw_buf_t draw_buf;


    lv_draw_buf_init(&draw_buf, LV_HOR_RES_MAX, LV_FB_LINE_NUM,
                     LV_COLOR_FORMAT_NATIVE,
                     LV_HOR_RES_MAX * (LV_COLOR_DEPTH >> 3),
                     &buf1_1, sizeof(buf1_1));

    rt_sem_init(&lcd_sema, "lv_lcd", 1, RT_IPC_FLAG_FIFO);
#if defined(LV_FB_TWO_NOT_SCREEN_SIZE)||defined(LV_FB_TWO_SCREEN_SIZE)
    static lv_draw_buf_t draw_buf2;

    lv_draw_buf_init(&draw_buf2, LV_HOR_RES_MAX, LV_FB_LINE_NUM,
                     LV_COLOR_FORMAT_NATIVE,
                     LV_HOR_RES_MAX * (LV_COLOR_DEPTH >> 3),
                     &buf1_2, sizeof(buf1_2));
    lv_display_set_draw_buffers(disp, &draw_buf, &draw_buf2);
#else
    lv_display_set_draw_buffers(disp, &draw_buf, NULL);
#endif /* LV_FB_TWO_NOT_SCREEN_SIZE ||  LV_FB_TWO_SCREEN_SIZE*/

#if defined(LV_FB_ONE_SCREEN_SIZE)||defined(LV_FB_TWO_SCREEN_SIZE)
    lv_display_set_render_mode(disp, LV_DISPLAY_RENDER_MODE_DIRECT);
#else
    lv_display_set_render_mode(disp, LV_DISPLAY_RENDER_MODE_PARTIAL);
#endif /* LV_FB_ONE_SCREEN_SIZE ||  LV_FB_TWO_SCREEN_SIZE*/

    uint32_t align_size = (info.draw_align != 0) ? info.draw_align : 1;
    int32_t offset_x = 0;
    int32_t offset_y = 0;

    /*Align disp to center of physical screen*/
    if (LV_HOR_RES_MAX < info.width)
    {
        offset_x = RT_ALIGN_DOWN((info.width - LV_HOR_RES_MAX) / 2, align_size);
    }

    if (LV_VER_RES_MAX < info.height)
    {
        offset_y = RT_ALIGN_DOWN((info.height - LV_VER_RES_MAX) / 2, align_size);
    }
    lv_display_set_offset(disp, offset_x, offset_y);
    lv_display_set_flush_cb(disp, lcd_flush);
    lv_display_set_flush_wait_cb(disp, wait_flush_done);
    lv_display_add_event_cb(disp, rounder_cb, LV_EVENT_INVALIDATE_AREA, &info);
    lv_display_add_event_cb(disp, render_start, LV_EVENT_REFR_START, NULL);

    lv_display_set_driver_data(disp, device);




#ifdef BSP_USING_LCD_FRAMEBUFFER
    drv_lcd_fb_init(name);
    update_fb();
#endif /* BSP_USING_LCD_FRAMEBUFFER */


    //disp_drv.monitor_cb = perf_monitor;



}

/*
    * @brief Check if the refreshing is done(Including LCD flushing, GPU rendering, etc.)
    * @return true if done, false if not done
*/
bool lv_refreshing_done(void)
{
    bool lcd_drawing;
    rt_device_control(device, RTGRAPHIC_CTRL_GET_BUSY, &lcd_drawing);
    if (lcd_drawing) return false;


    if (drv_epic_is_busy())
    {
        return false;
    }

    return true;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
