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
#include "rtconfig.h"
#include "littlevgl2rtt.h"
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

#include "lvsf_perf.h"

#include "cpu_usage_profiler.h"

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


#define LCD_FLUSH_EXP_MS   (5000)//Include LCD reset time
#define LCD_FLUSH_DEBUG


#ifdef FRAME_BUFFER_IN_PSRAM
    #define FRAME_BUFFER_BSS_SECT_BEGIN(frambuf) L2_NON_RET_BSS_SECT_BEGIN(frambuf)
    #define FRAME_BUFFER_BSS_SECT_END L2_NON_RET_BSS_SECT_END
    #define FRAME_BUFFER_BSS_SECT(frambuf, var) L2_NON_RET_BSS_SECT(frambuf, var)
#else
    #define FRAME_BUFFER_BSS_SECT_BEGIN(frambuf) L1_NON_RET_BSS_SECT_BEGIN(frambuf)
    #define FRAME_BUFFER_BSS_SECT_END L1_NON_RET_BSS_SECT_END
    #define FRAME_BUFFER_BSS_SECT(frambuf, var) L1_NON_RET_BSS_SECT(frambuf, var)
#endif

extern void perf_monitor(lv_disp_drv_t *disp_drv, uint32_t time, uint32_t px);
void lv_disp_buf_init3(lv_disp_draw_buf_t *disp_buf, void *buf1, void *buf2, uint16_t width, uint16_t height, uint8_t cf);
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


static lv_disp_drv_t *lcd_flushing_disp_drv = NULL;
static lv_disp_drv_t disp_drv;

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

    if ((16 == LV_COLOR_DEPTH) && (LV_COLOR_16_SWAP))
        cf = RTGRAPHIC_PIXEL_FORMAT_RGB565P;
    else if (16 == LV_COLOR_DEPTH)
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

    fb_dsc.area.x0 = disp_drv.offset_x;
    fb_dsc.area.y0 = disp_drv.offset_y;
    fb_dsc.area.x1 = disp_drv.offset_x + disp_drv.hor_res - 1;
    fb_dsc.area.y1 = disp_drv.offset_y + disp_drv.ver_res - 1;
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
static void rounder_cb(lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    uint32_t align_size = (info.draw_align != 0) ? info.draw_align : 1;

    area->x1 = RT_ALIGN_DOWN(area->x1, align_size);
    area->x2 = RT_ALIGN(area->x2 + 1, align_size) - 1;

    area->y1 = RT_ALIGN_DOWN(area->y1, align_size);
    area->y2 = RT_ALIGN(area->y2 + 1, align_size) - 1;


#if defined(FB_CMPR_RATE)
    /*Extend to whole line if FB compression is enabled.*/
    area->x1 = 0;
    area->x2 = LV_HOR_RES_MAX - 1;
#endif

}



static void render_start(lv_disp_drv_t *disp_drv)
{


#if defined(LCD_FB_USING_TWO_COMPRESSED)||defined(LCD_FB_USING_TWO_UNCOMPRESSED)
    switch_draw_buf();
    update_fb();
#endif /* LCD_FB_USING_TWO_COMPRESSED ||  LCD_FB_USING_TWO_UNCOMPRESSED*/

}



static void wait_flush_done(lv_disp_drv_t *disp_drv)
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
    lv_disp_drv_t *p_disp = lcd_flushing_disp_drv;
    lcd_flushing_disp_drv = NULL;

    rt_err_t err;
    err = rt_sem_release(&lcd_sema);
    RT_ASSERT(RT_EOK == err);

    /* Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(p_disp);
}
#else
static rt_err_t lcd_flush_done(rt_device_t dev, void *buffer)
{
    lv_disp_drv_t *p_disp = lcd_flushing_disp_drv;
    lcd_flushing_disp_drv = NULL;
    debug_lcd_flush_end();

    rt_err_t err;
    err = rt_sem_release(&lcd_sema);
    RT_ASSERT(RT_EOK == err);

    /* Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(p_disp);

    return RT_EOK;
}

#endif /* BSP_USING_LCD_FRAMEBUFFER */

void lcd_flush(lv_disp_drv_t *disp_drv, const lv_area_t *buf_area, lv_color_t *color_p)
{
    LCD_AreaDef clip_area =   //Buf clip area
    {
        .x0 = disp_drv->draw_ctx->clip_area->x1 + disp_drv->offset_x,
        .y0 = disp_drv->draw_ctx->clip_area->y1 + disp_drv->offset_y,
        .x1 = disp_drv->draw_ctx->clip_area->x2 + disp_drv->offset_x,
        .y1 = disp_drv->draw_ctx->clip_area->y2 + disp_drv->offset_y
    };

    LCD_AreaDef src_area =
    {
        .x0 = buf_area->x1,
        .y0 = buf_area->y1,
        .x1 = buf_area->x2,
        .y1 = buf_area->y2
    };

    //BUG 1944: Invalid coordinates if framebuffer larger than LCD and refresh area out of screen
    if ((clip_area.x0 > clip_area.x1) || (clip_area.y0 > clip_area.y1))
    {
        /* Inform the graphics library that you are ready with the flushing*/
        lv_disp_flush_ready(disp_drv);
        return;
    }

    lcd_flushing_disp_drv = disp_drv;





    rt_err_t err;
    err = rt_sem_take(&lcd_sema, rt_tick_from_millisecond(LCD_FLUSH_EXP_MS));
    RT_ASSERT(RT_EOK == err);
#ifdef BSP_USING_LCD_FRAMEBUFFER
    drv_lcd_fb_write_send(&clip_area, &src_area, (uint8_t *)color_p, lcd_flush_done,
                          disp_drv->draw_buf->flushing_last);
#else
    debug_lcd_flush_start(clip_area.x0, clip_area.y0, clip_area.x1, clip_area.y1);
    rt_device_set_tx_complete(device, lcd_flush_done);
    rt_graphix_ops(device)->set_window(clip_area.x0, clip_area.y0, clip_area.x1, clip_area.y1);
    rt_graphix_ops(device)->draw_rect_async((const char *)color_p, src_area.x0,
                                            src_area.y0, src_area.x1, src_area.y1);
#endif /* BSP_USING_LCD_FRAMEBUFFER */

}



#if defined(LV_FB_ONE_SCREEN_SIZE) && defined(LCD_FB_USING_NONE)
/**
 * @brief Get the writeable area in refreshing area
 * @param a_in - refreshing area, origin is framebuffer'TL
 * @param a_out - the intersect of writeable area and rereshing area
 */
void wait_writeable_area(const lv_area_t *a_in, lv_area_t *a_out)
{
    LCD_AreaDef writeable_area;

    writeable_area.x0 = a_in->x1;
    writeable_area.y0 = a_in->y1;
    writeable_area.x1 = a_in->x2;
    writeable_area.y1 = a_in->y2;

    drv_lcd_fb_get_write_area(&writeable_area, rt_tick_from_millisecond(LCD_FLUSH_EXP_MS));

    a_out->x1 = writeable_area.x0;
    a_out->y1 = writeable_area.y0;
    a_out->x2 = writeable_area.x1;
    a_out->y2 = writeable_area.y1;

}
#endif



static size_t  gauss_alloced = 0;
void *gauss_buf_alloc(uint32_t type, size_t size)
{
    uint8_t *ret_p;

    if (gauss_alloced + RT_ALIGN(size, 4) > sizeof(buf1_1)) return NULL;

    ret_p = ((uint8_t *)&buf1_1[0]) + gauss_alloced;

    gauss_alloced += RT_ALIGN(size, 4);

    RT_ASSERT(0 == (((uint32_t)ret_p) & 0x3));
    return (void *)ret_p;
}

void gauss_buf_free(uint32_t type, void *p)
{
    (void) type;
    (void) p;

    gauss_alloced = 0;
}


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




void *get_disp_buf(uint32_t size)
{
    lv_disp_draw_buf_t *draw_buf = lv_disp_get_draw_buf(lv_disp_get_default());
    uint32_t buf_size = draw_buf->size * LV_COLOR_DEPTH / 8;
    if (buf_size >= size)
    {
        return (void *) draw_buf->buf_act;
    }

#if defined(LV_FB_TWO_NOT_SCREEN_SIZE) || defined(LV_FB_TWO_SCREEN_SIZE)
    //two buffer: buf1_1 and buf1_2 exist
    if (size <= 2 * buf_size && draw_buf->buf1 + buf_size == draw_buf->buf2)
    {
        return (void *) draw_buf->buf1;
    }
#endif

#ifndef LCD_FB_USING_NONE
    if (size <= sizeof(buf2_1))
        return (void *) get_draw_buf();
#endif /* LCD_FB_USING_NONE */


    return NULL;
}


void *get_display_frame_buf(uint32_t *size)
{
    *size = sizeof(buf1_1);
    return (void *)buf1_1;
}


void lv_disp_buf_init3(lv_disp_draw_buf_t *disp_buf, void *buf1, void *buf2, uint16_t width, uint16_t height, uint8_t cf)
{
    LV_UNUSED(cf);
    lv_disp_draw_buf_init(disp_buf, buf1, buf2, width * height);
    disp_buf->cf = cf;
    disp_buf->width = width;
    disp_buf->height = height;
}




//LV_ATTRIBUTE_MEM_ALIGN static lv_color_t buf1_1[LV_HOR_RES_MAX * 10];
lv_disp_drv_t *lv_lcd_init(const char *name)
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
    if ((16 == LV_COLOR_DEPTH) && (LV_COLOR_16_SWAP))
        cf = RTGRAPHIC_PIXEL_FORMAT_RGB565P;
    else if (16 == LV_COLOR_DEPTH)
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
    rt_sem_init(&lcd_sema, "lv_lcd", 1, RT_IPC_FLAG_FIFO);
    static lv_disp_draw_buf_t disp_buf;
#if defined(LV_FB_TWO_NOT_SCREEN_SIZE)||defined(LV_FB_TWO_SCREEN_SIZE)
    lv_disp_buf_init3(&disp_buf, (void *)buf1_1, (void *)buf1_2, FB_ALIGNED_HOR_RES, LV_FB_LINE_NUM, LV_IMG_CF_TRUE_COLOR);
#else
    lv_disp_buf_init3(&disp_buf, (void *)buf1_1, (void *)NULL, FB_ALIGNED_HOR_RES, LV_FB_LINE_NUM, LV_IMG_CF_TRUE_COLOR);
#endif /* LV_FB_TWO_NOT_SCREEN_SIZE ||  LV_FB_TWO_SCREEN_SIZE*/

    lv_disp_drv_init(&disp_drv);
#if defined(LV_FB_ONE_SCREEN_SIZE)||defined(LV_FB_TWO_SCREEN_SIZE)
    disp_drv.direct_mode  = 1;
#else
    disp_drv.direct_mode  = 0;
#endif /* LV_FB_ONE_SCREEN_SIZE ||  LV_FB_TWO_SCREEN_SIZE*/

    uint32_t align_size = (info.draw_align != 0) ? info.draw_align : 1;
    disp_drv.hor_res          = LV_HOR_RES_MAX;
    disp_drv.ver_res          = LV_VER_RES_MAX;

    /*Align disp to center of physical screen*/
    if (disp_drv.hor_res < info.width)
    {
        disp_drv.offset_x = RT_ALIGN_DOWN((info.width - disp_drv.hor_res) / 2, align_size);
    }

    if (disp_drv.ver_res < info.height)
    {
        disp_drv.offset_y = RT_ALIGN_DOWN((info.height - disp_drv.ver_res) / 2, align_size);
    }


    disp_drv.draw_buf = &disp_buf;

    disp_drv.monitor_cb = perf_monitor;
    disp_drv.flush_cb = lcd_flush;
    disp_drv.wait_cb = wait_flush_done;

    disp_drv.rounder_cb = rounder_cb;
    disp_drv.render_start_cb = render_start;

#ifdef BSP_USING_LCD_FRAMEBUFFER
    drv_lcd_fb_init(name);
    update_fb();
#endif /* BSP_USING_LCD_FRAMEBUFFER */


    return &disp_drv;
}


uint8_t fb_get_cmpr_rate(void)
{
    uint8_t cmpr_rate;
    rt_device_control(device, SF_GRAPHIC_CTRL_LCDC_CP2COMPRESS_FB_AND_SEND_CMPRATE_GET, &cmpr_rate);

    return cmpr_rate;
}

uint8_t fb_set_cmpr_rate(uint8_t rate)
{
    if (rate > 9)
    {
        rate = 0;
    }
    uint32_t cmpr_size;
    uint32_t target_size;
    uint8_t cmpr_rate;
    //HAL_EXT_DMA_CalcCompressedSize(240*240*2/4, cmpr_rate, 240, &cmpr_size, &target_size);
    //rt_kprintf("compress_rate:%d,%d\n",cmpr_rate,target_size);
    cmpr_rate = rate;
    rt_device_control(device, SF_GRAPHIC_CTRL_LCDC_CP2COMPRESS_FB_AND_SEND_CMPRATE_SET, &cmpr_rate);

    return rate;
}

/* This function only for BSP_USING_RAMLESS_LCD*/
uint8_t *get_disp_buf_in_turn(uint32_t size)
{
    lv_disp_draw_buf_t *draw_buf = lv_disp_get_draw_buf(lv_disp_get_default());
    RT_ASSERT(draw_buf->size * LV_COLOR_DEPTH / 8 >= size);

    if (draw_buf->buf1 && draw_buf->buf2)
    {
        if (draw_buf->buf1 == draw_buf->buf_act)
            draw_buf->buf_act =  draw_buf->buf2;
        else
            draw_buf->buf_act = draw_buf->buf1;
    }

    memset(draw_buf->buf_act, 0x00, size);

    return draw_buf->buf_act;
}

bool lv_lcd_draw_error()
{
    uint8_t err;
    if (RT_EOK == rt_device_control(device, SF_GRAPHIC_CTRL_GET_DRAW_ERR, &err))
    {
        return (1 == err);
    }
    else
    {
        return false;
    }
}
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
