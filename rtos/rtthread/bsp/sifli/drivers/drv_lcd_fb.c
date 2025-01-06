/**
  ******************************************************************************
  * @file   drv_lcd_fb.c
  * @author Sifli software development team
  * @brief  Enables the Display.
  *
* *****************************************************************************
**/
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

#include <rtthread.h>
#include <rtdevice.h>
#include "stdlib.h"
#include "drv_common.h"
#include "drv_lcd_private.h"
#include "drv_lcd_fb.h"
#include "drv_ext_dma.h"
#include "mem_section.h"
#include "string.h"
#ifdef BSP_USING_EPIC
    #include "drv_epic.h"
#endif /* BSP_USING_EPIC */

//#define ENABLE_GP_DMA_COPY //Copy with normal DMA
#ifdef BSP_USING_HW_AES
    #define ENABLE_AES_COPY   //Use AES as memcpy
#endif /* BSP_USING_HW_AES */
//#define DRV_LCD_FB_STATISTICS

#define  DBG_LEVEL            DBG_INFO  //DBG_LOG //

#define LOG_TAG                "drv.lcd_fb"
#include "log.h"

#define FB_COPY_EXP_MS   (1000)
#define FB_FLUSH_EXP_MS   (5000)
#define AreaString "x0y0x1y1=[%d,%d,%d,%d]"
#define AreaParams(area) (area)->x0,(area)->y0,(area)->x1,(area)->y1
#ifdef ENABLE_GP_DMA_COPY
    //#define GP_DMA_CHANNEL   DMA1_Channel3
    //#define GP_DMA_IRQn      DMAC1_CH3_IRQn
    //#define GP_DMA_IRQHandler DMAC1_CH3_IRQHandler
    #error "Need to allocate DMA channel automatically!"
#endif /* ENABLE_GP_DMA_COPY */

#ifdef ENABLE_AES_COPY
    #include "drv_aes.h"
#endif /* ENABLE_AES_COPY */
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
enum
{
    EVENT_LINE_VALID        = (1 << 0),
    EVENT_FLUSH_FB_DONE     = (1 << 1),
    EVENT_WRITE_DONE        = (1 << 2)
};
#define EVENT_ALL_DONE (EVENT_LINE_VALID|EVENT_FLUSH_FB_DONE|EVENT_WRITE_DONE)

typedef void (*dma_write_cbk)(void);

typedef struct
{
    rt_device_t p_lcd_dev;
    struct rt_device_graphic_info lcd_info;

    lcd_fb_desc_t  fb;     /*Framebuffer description*/
    LCD_AreaDef window;    /*LCD recieve area, origin is LCD's TL*/

    uint8_t  flushing_lcd;

    //Writeable lines [0 ~ scheme6_valid_y1],  if 'scheme6_valid_y1' is -1 means no valid lines
    int32_t scheme6_valid_y1;
    //Set windows's y0
    int32_t scheme6_y0;


    struct rt_event  event;

    write_fb_cbk cb;


#ifdef CHECK_FB_WRITE_OVERFLOW
    uint8_t *overwrite_check_addr;
    uint8_t overwrite_check_golden[4];
#endif /* CHECK_FB_WRITE_OVERFLOW */

    uint32_t write_start_tick;       /*Last aysnc write framebuffer start tick*/
    uint32_t write_end_tick;         /*Last aysnc write framebuffer end tick*/

    uint32_t flush_start_tick;       /*Last aysnc flush LCD start tick*/
    uint32_t flush_end_tick;         /*Last aysnc flush LCD end tick*/
#ifdef DRV_LCD_FB_STATISTICS
    uint32_t write_ticks_sum;
    uint32_t write_bytes_sum;

    uint32_t epic_copy_cnt;
    uint32_t gpdma_copy_cnt;
    uint32_t extdma_copy_cnt;
    uint32_t aes_copy_cnt;
#endif /* DRV_LCD_FB_STATISTICS */

    uint32_t dbg_write_req;
    uint32_t dbg_write_rsp;
    uint32_t dbg_flush_req;
    uint32_t dbg_flush_rsp;

    uint8_t dma_faster_than_lcdc;

#ifdef ENABLE_GP_DMA_COPY
    DMA_HandleTypeDef testdma;
    uint32_t src;
    uint32_t dst;
    uint32_t left_counts;
    dma_write_cbk  dma_cb;
#endif /* ENABLE_GP_DMA_COPY */

#ifdef ENABLE_AES_COPY
    uint32_t src;
    uint32_t dst;
    dma_write_cbk  dma_cb;
#endif /* ENABLE_AES_COPY */

} LCD_FBTypeDef;


static LCD_FBTypeDef drv_lcd_fb;
static const LCD_AreaDef  invalid_area = {-1, -1, -2, -2};

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


#ifdef BSP_USING_LCD_FRAMEBUFFER

static bool area_intersect(LCD_AreaDef *res_p, const LCD_AreaDef *a0_p, const LCD_AreaDef *a1_p)
{
    /* Get the smaller area from 'a0_p' and 'a1_p' */
    res_p->x0 = HAL_MAX(a0_p->x0, a1_p->x0);
    res_p->y0 = HAL_MAX(a0_p->y0, a1_p->y0);
    res_p->x1 = HAL_MIN(a0_p->x1, a1_p->x1);
    res_p->y1 = HAL_MIN(a0_p->y1, a1_p->y1);

    /*If x0 or y0 greater then x1 or y1 then the areas union is empty*/
    bool union_ok = true;
    if ((res_p->x0 > res_p->x1) || (res_p->y0 > res_p->y1))
    {
        union_ok = false;
    }

    return union_ok;
}

static bool is_area_valid(const LCD_AreaDef *a0_p)
{
    return ((a0_p->x0 <= a0_p->x1) && (a0_p->y0 <= a0_p->y1));
}

static void LCD_area_to_EPIC_area(const LCD_AreaDef *lcd_a, EPIC_AreaTypeDef *epic_a)
{
    epic_a->x0 = (int16_t)lcd_a->x0;
    epic_a->x1 = (int16_t)lcd_a->x1;
    epic_a->y0 = (int16_t)lcd_a->y0;
    epic_a->y1 = (int16_t)lcd_a->y1;
}

static void err_debug(void)
{
    LOG_E("===err_debug===");
    LOG_E("Fb=%x area:"AreaString" fmt=%d,cmpr=%d,lineBytes=%d", drv_lcd_fb.fb.p_data,
          AreaParams(&drv_lcd_fb.fb.area),
          drv_lcd_fb.fb.format, drv_lcd_fb.fb.cmpr_rate,
          drv_lcd_fb.fb.line_bytes);

    LOG_E("flushing_lcd=%d", drv_lcd_fb.flushing_lcd,
          drv_lcd_fb.scheme6_y0,
          drv_lcd_fb.scheme6_valid_y1);
    LOG_E("window:"AreaString, AreaParams(&drv_lcd_fb.window));
    LOG_E("flush_req=%d,rsp=%d, write_req=%d,rsp=%d",
          drv_lcd_fb.dbg_flush_req, drv_lcd_fb.dbg_flush_rsp,
          drv_lcd_fb.dbg_write_req, drv_lcd_fb.dbg_write_rsp);
    LOG_E("event=%x", drv_lcd_fb.event.set);
}
#define DRV_LCD_FB_ASSERT(expr) do{if(!(expr)){err_debug();RT_ASSERT(0);}}while(0)


static void set_valid_y(int32_t y)
{
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    drv_lcd_fb.scheme6_valid_y1 = y;
    rt_err_t err;
    err = rt_event_send(&drv_lcd_fb.event, EVENT_LINE_VALID);
    RT_ASSERT(RT_EOK == err);
    rt_hw_interrupt_enable(level);

    LOG_D("drv_lcd_fb.scheme6_valid_y1 %d", y);
}

static rt_err_t wait_line_valid(LCD_AreaDef *write_area, int32_t wait_ms)
{
    rt_err_t err = RT_EOK;
    //Wait fb writebale
    while (drv_lcd_fb.scheme6_valid_y1 < write_area->y1)
    {
        err = rt_event_recv(&drv_lcd_fb.event, EVENT_LINE_VALID,
                            RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                            rt_tick_from_millisecond(wait_ms), NULL);

        if (RT_EOK != err) break; //Overwrite anyway
    }

    return err;
}

void HAL_LCDC_SendLineCpltCbk(LCDC_HandleTypeDef *lcdc, uint32_t line)
{
    /*
        Since LCD device invoked by others, and others can't overwrite 'HAL_LCDC_SendLineCpltCbk',
        so 'drv_lcd_fb.scheme6_valid_y1' will be an invalid value('LV_VER_RES_MAX - interval_lines' for example).

        It can cause an assertion as lvgl can't wait util 'drv_lcd_fb.scheme6_valid_y1' equals 'LV_VER_RES_MAX - 1'.
    */
    if (drv_lcd_fb.flushing_lcd)
    {
        RT_ASSERT(drv_lcd_fb.scheme6_y0 != INT32_MIN);
        LOG_D("SendLineCpltCbk %d \r\n", drv_lcd_fb.scheme6_y0 + line);
        set_valid_y(drv_lcd_fb.scheme6_y0 + line);
    }

}
#ifdef PKG_USING_SYSTEMVIEW
#include "SEGGER_SYSVIEW.h"
#define COPY_FB_SYSTEMVIEW_MARK_ID   0xC094FABF
#define FLUSH_LCD_SYSTEMVIEW_MARK_ID 0xBBBBBBBB
static void SystemView_mark_start(uint32_t id, const char *desc, ...)
{
    va_list args;
    static char rt_log_buf[128];

    va_start(args, desc);
    rt_vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, desc, args);
    SEGGER_SYSVIEW_OnUserStart(id);
    SEGGER_SYSVIEW_Print(&rt_log_buf[0]);
    va_end(args);
}

static void SystemView_mark_stop(uint32_t id)
{
    SEGGER_SYSVIEW_OnUserStop(id);
}
#endif

static rt_err_t fb_flush_done(rt_device_t dev, void *buffer)
{
    rt_err_t err;

    drv_lcd_fb.flush_end_tick = rt_tick_get();
    drv_lcd_fb.dbg_flush_rsp++;
    LOG_D("fb_flush_done %p, cost=%d ticks", buffer, drv_lcd_fb.flush_end_tick - drv_lcd_fb.flush_start_tick);
#ifdef PKG_USING_SYSTEMVIEW
    SystemView_mark_stop(FLUSH_LCD_SYSTEMVIEW_MARK_ID);
#endif /* PKG_USING_SYSTEMVIEW */

    if (drv_lcd_fb.fb.p_data == buffer)
    {
        drv_lcd_fb.flushing_lcd = 0;
        drv_lcd_fb.scheme6_y0 = INT32_MIN;
        set_valid_y(drv_lcd_fb.fb.area.y1);
    }
    err = rt_event_send(&drv_lcd_fb.event, EVENT_FLUSH_FB_DONE);

    return err;
}

static rt_err_t fb_flush_start(void)
{
    rt_err_t err;
    rt_device_t p_lcd_dev = drv_lcd_fb.p_lcd_dev;
    LCD_AreaDef *p_win_area = &drv_lcd_fb.window;
    LCD_AreaDef *p_fb_area = &drv_lcd_fb.fb.area;
    LCD_AreaDef common_area;/*Relative area of FB will be flushed*/


    if (area_intersect(&common_area, p_win_area, p_fb_area))
    {
        lcd_flush_info_t flush_info;
        set_valid_y(common_area.y0 - p_fb_area->y0 - 1);
        drv_lcd_fb.scheme6_y0 = common_area.y0 - p_fb_area->y0;


        drv_lcd_fb.flushing_lcd = 1;
        drv_lcd_fb.flush_start_tick = rt_tick_get();
        drv_lcd_fb.dbg_flush_req++;

        //Flush to lcd
        rt_device_set_tx_complete(drv_lcd_fb.p_lcd_dev, fb_flush_done);

        flush_info.cmpr_rate = drv_lcd_fb.fb.cmpr_rate;
        flush_info.pixel      = drv_lcd_fb.fb.p_data;
        flush_info.color_format    = drv_lcd_fb.fb.format;
        memcpy(&flush_info.window, &drv_lcd_fb.window, sizeof(flush_info.window));
        memcpy(&flush_info.pixel_area, &drv_lcd_fb.fb.area, sizeof(flush_info.pixel_area));

#ifdef PKG_USING_SYSTEMVIEW
        SystemView_mark_start(FLUSH_LCD_SYSTEMVIEW_MARK_ID,
                              "window:"AreaString" p_data=%p", AreaParams(&flush_info.window), flush_info.pixel);
#endif /* PKG_USING_SYSTEMVIEW */
        LOG_D("fb_flush_start window:"AreaString" fb:"AreaString" p_data=%p",
              AreaParams(&flush_info.window), AreaParams(&flush_info.pixel_area), flush_info.pixel);

        err = rt_device_control(p_lcd_dev, SF_GRAPHIC_CTRL_LCDC_FLUSH, &flush_info);

        if (RT_EOK != err)  LOG_E("fb_flush_start err=%d", err);

    }
    else
    {
        LOG_D("NoIntersect window:"AreaString" fb:"AreaString" p_data=%p",
              AreaParams(p_fb_area), AreaParams(p_win_area), drv_lcd_fb.fb.p_data);
        fb_flush_done(drv_lcd_fb.p_lcd_dev, drv_lcd_fb.fb.p_data);
    }


    //Mark current window is flushed.
    memcpy(&drv_lcd_fb.window, &invalid_area, sizeof(LCD_AreaDef));

    return err;
}


static void write_fb_cb1(void)
{
    drv_lcd_fb.write_end_tick = rt_tick_get();
    drv_lcd_fb.dbg_write_rsp++;
#ifdef DRV_LCD_FB_STATISTICS
    drv_lcd_fb.write_ticks_sum += drv_lcd_fb.write_end_tick - drv_lcd_fb.write_start_tick;
#endif /* DRV_LCD_FB_STATISTICS */

#ifdef CHECK_FB_WRITE_OVERFLOW
    //Is there overflow?
    if (0 != memcmp(&drv_lcd_fb.overwrite_check_golden[0],
                    drv_lcd_fb.overwrite_check_addr,
                    sizeof(drv_lcd_fb.overwrite_check_golden)))
    {
        LOG_W("Overwrite!! %x%x%x%x,%x%x%x%x,",
              drv_lcd_fb.overwrite_check_golden[0],
              drv_lcd_fb.overwrite_check_golden[1],
              drv_lcd_fb.overwrite_check_golden[2],
              drv_lcd_fb.overwrite_check_golden[3],
              drv_lcd_fb.overwrite_check_addr[0],
              drv_lcd_fb.overwrite_check_addr[1],
              drv_lcd_fb.overwrite_check_addr[2],
              drv_lcd_fb.overwrite_check_addr[3]
             );
    }
#endif /* CHECK_FB_WRITE_OVERFLOW */

#ifndef BSP_USE_LCDC2_ON_HPSYS
#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_stop();
#endif  /* RT_USING_PM */
#endif /* BSP_USE_LCDC2_ON_HPSYS */
}

static void write_fb_cb2(void)
{
    rt_err_t err;

    write_fb_cbk cb = drv_lcd_fb.cb;
    drv_lcd_fb.cb = NULL;

#ifdef PKG_USING_SYSTEMVIEW
    SystemView_mark_stop(COPY_FB_SYSTEMVIEW_MARK_ID);
#endif /* PKG_USING_SYSTEMVIEW */

    if (cb) cb(&drv_lcd_fb.fb);

    err = rt_event_send(&drv_lcd_fb.event, EVENT_WRITE_DONE);
    RT_ASSERT(RT_EOK == err);
}

static void write_fb_cb_done(void)
{
    write_fb_cb1();
    write_fb_cb2();
}


static void write_fb_cb_done_send(void)
{
    write_fb_cb1();
    fb_flush_start();
    write_fb_cb2();
}



static void write_fb_err_cb(void)
{
    LOG_E("write_fb_err_cb extdma error(%x).", EXT_DMA_GetError());

    //RT_ASSERT(0);
}

#ifdef ENABLE_GP_DMA_COPY
static void DMA_reload(void)
{
    LOG_D("DMA_reload=0x%x", drv_lcd_fb.left_counts);

#define  max_counts  0xFFFFU
    if (drv_lcd_fb.left_counts > max_counts)
    {
        uint32_t src_addr = drv_lcd_fb.src;
        uint32_t dst_addr = drv_lcd_fb.dst;
        uint32_t offset;

        if (DMA_MDATAALIGN_WORD == drv_lcd_fb.testdma.Init.MemDataAlignment)
            offset = max_counts << 2;
        else if (DMA_MDATAALIGN_HALFWORD == drv_lcd_fb.testdma.Init.MemDataAlignment)
            offset = max_counts << 1;
        else if (DMA_MDATAALIGN_BYTE == drv_lcd_fb.testdma.Init.MemDataAlignment)
            offset = max_counts;

        drv_lcd_fb.left_counts -= max_counts;
        drv_lcd_fb.src = src_addr + offset;
        drv_lcd_fb.dst = dst_addr + offset;

        HAL_DMA_RegisterCallback(&drv_lcd_fb.testdma, HAL_DMA_XFER_CPLT_CB_ID, (void (*)(struct __DMA_HandleTypeDef *))DMA_reload);
        HAL_DMA_Start_IT(&drv_lcd_fb.testdma, src_addr, dst_addr, max_counts);

    }
    else
    {
        uint32_t counts = drv_lcd_fb.left_counts;
        drv_lcd_fb.left_counts = 0;
        HAL_DMA_RegisterCallback(&drv_lcd_fb.testdma, HAL_DMA_XFER_CPLT_CB_ID, (void (*)(struct __DMA_HandleTypeDef *))drv_lcd_fb.dma_cb);
        HAL_DMA_Start_IT(&drv_lcd_fb.testdma, drv_lcd_fb.src, drv_lcd_fb.dst, counts);
    }
}

void GP_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    LOG_D("GP_DMA_IRQHandler=0x%x, cb=%p, cb2=%p", drv_lcd_fb.left_counts, drv_lcd_fb.dma_cb, drv_lcd_fb.testdma.XferCpltCallback);

    HAL_DMA_IRQHandler(&drv_lcd_fb.testdma);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* ENABLE_GP_DMA_COPY */

#ifdef ENABLE_AES_COPY
void AES_CopyCb(void)
{
    LOG_D("AES_CopyCb, cb=%p", drv_lcd_fb.dma_cb);

    if (drv_lcd_fb.dma_cb)
    {
        dma_write_cbk  cb = drv_lcd_fb.dma_cb;
        drv_lcd_fb.dma_cb = NULL;

        if (cb) cb();
    }
}
#endif /* ENABLE_AES_COPY */

#ifdef DRV_LCD_FB_STATISTICS
static void print_statistics(void)
{
    uint32_t cost_ms = drv_lcd_fb.write_ticks_sum * (1000u / RT_TICK_PER_SECOND);

    LOG_I("avg %dKB/s. epic:%d,gpdma:%d,extdma:%d,aes:%d", drv_lcd_fb.write_bytes_sum / cost_ms,
          drv_lcd_fb.epic_copy_cnt,
          drv_lcd_fb.gpdma_copy_cnt,
          drv_lcd_fb.extdma_copy_cnt,
          drv_lcd_fb.aes_copy_cnt);

    //Reset statistics variables.
    drv_lcd_fb.write_ticks_sum = 0;
    drv_lcd_fb.write_bytes_sum = 0;
    drv_lcd_fb.epic_copy_cnt = 0;
    drv_lcd_fb.gpdma_copy_cnt = 0;
    drv_lcd_fb.extdma_copy_cnt = 0;
    drv_lcd_fb.aes_copy_cnt = 0;
}
#endif /* DRV_LCD_FB_STATISTICS */
/**
 * @brief Copy part of src to current FB(drv_lcd_fb.fb)
 * @param clip_area - Relative area of src
 * @param src_area -
 * @param src -
 * @param cb - Write callback
 * @return
 */
static rt_err_t write_fb_async(LCD_AreaDef *clip_area, LCD_AreaDef *src_area, const uint8_t *src, dma_write_cbk cb)
{
    rt_err_t err =  RT_EOK;
    uint8_t  use_extdma = 0;
    uint8_t  use_gp_dma = 0;
    uint8_t  use_aes = 0;
    LCD_AreaDef *dst_area = &drv_lcd_fb.fb.area;

    //src_line_addr, dst_line_addr, len are all after clipped
    uint32_t src_line_addr, dst_line_addr, len, bytes_per_pixel;
    uint32_t width, height, src_width;


    switch (drv_lcd_fb.fb.format)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        bytes_per_pixel  = 2;
        break;
    case RTGRAPHIC_PIXEL_FORMAT_RGB888:
        bytes_per_pixel = 3;
        break;

    default:
        RT_ASSERT(0);
        break;
    }

    src_width = src_area->x1 - src_area->x0 + 1;
    width  = clip_area->x1  - clip_area->x0 + 1;
    height = clip_area->y1  - clip_area->y0 + 1;


    src_line_addr = ((uint32_t)src)
                    + ((clip_area->y0 - src_area->y0) * src_width) * bytes_per_pixel;

    dst_line_addr = ((uint32_t)drv_lcd_fb.fb.p_data)
                    + (clip_area->y0 - drv_lcd_fb.fb.area.y0) * drv_lcd_fb.fb.line_bytes;
    len = bytes_per_pixel * width * height;

    LOG_D("clip area:"AreaString" src area:"AreaString" src=%p", AreaParams(clip_area), AreaParams(src_area), src);
    LOG_D("src_line_addr=0x%x dst_line_addr=0x%x len=0x%x", src_line_addr, dst_line_addr, len);

    if (src_line_addr == dst_line_addr)
    {
        LOG_D("src_line_addr==dst_line_addr skip.");
        cb();
        return RT_EOK;
    }

    if (((clip_area->x0 == src_area->x0) && (dst_area->x0 == src_area->x0)
            && (clip_area->x1 == src_area->x1) && (dst_area->x1 == src_area->x1)))
    {
        //Continuous buffer copy
        use_extdma = 1;
#ifdef ENABLE_GP_DMA_COPY
        use_gp_dma    = 1;
#endif /* ENABLE_GP_DMA_COPY */
#ifdef ENABLE_AES_COPY
        use_aes = 1;
#endif /* ENABLE_AES_COPY */
    }

#if defined(BSP_LCDC_USING_DPI)  && defined(LCD_HOR_RES_MAX)
#if (LCD_HOR_RES_MAX > LCDC_DPI_MAX_WIDTH)
    use_extdma = 0; //EXTDMA is occupied by LCDC DPI_AUX mode
#endif
#endif /* BSP_LCDC_USING_DPI */

    if (drv_lcd_fb.fb.cmpr_rate != 0)
    {
        use_gp_dma = 0; //GP_DMA does not support compressed buf
        use_aes = 0;
    }

#ifdef PKG_USING_SYSTEMVIEW
    SystemView_mark_start(COPY_FB_SYSTEMVIEW_MARK_ID,
                          "area:"AreaString" src=%p", AreaParams(clip_area), src);
#endif /* PKG_USING_SYSTEMVIEW */



#ifdef DRV_LCD_FB_STATISTICS
    if (drv_lcd_fb.write_ticks_sum > 2000) print_statistics();
    drv_lcd_fb.write_bytes_sum += len;
#endif /* DRV_LCD_FB_STATISTICS */
    drv_lcd_fb.write_start_tick = rt_tick_get();
    drv_lcd_fb.dbg_write_req++;

#ifdef CHECK_FB_WRITE_OVERFLOW
    //Read some bytes right after framebufer to 'overwrite_check_golden'
    drv_lcd_fb.overwrite_check_addr = (uint8_t *)(drv_lcd_fb.fb.p_data +
                                      drv_lcd_fb.fb.line_bytes * (drv_lcd_fb.fb.area.y1 - drv_lcd_fb.fb.area.y0 + 1));
    mpu_dcache_clean((void *)drv_lcd_fb.overwrite_check_addr, sizeof(drv_lcd_fb.overwrite_check_golden));
    memcpy(&drv_lcd_fb.overwrite_check_golden[0],
           drv_lcd_fb.overwrite_check_addr,
           sizeof(drv_lcd_fb.overwrite_check_golden));
#endif /* CHECK_FB_WRITE_OVERFLOW */



#ifndef BSP_USE_LCDC2_ON_HPSYS
#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_start();
#endif  /* RT_USING_PM */
#endif /* BSP_USE_LCDC2_ON_HPSYS */
    if (use_extdma || use_gp_dma || use_aes)
    {
        if (0)
        {
        }
#ifdef ENABLE_GP_DMA_COPY
        else if (use_gp_dma)
        {
            uint32_t counts;

            drv_lcd_fb.testdma.Instance = GP_DMA_CHANNEL;
            drv_lcd_fb.testdma.Init.Request = 0; //DMA_REQUEST_MEM2MEM;
            drv_lcd_fb.testdma.Init.Direction = DMA_MEMORY_TO_MEMORY;
            drv_lcd_fb.testdma.Init.PeriphInc = DMA_PINC_ENABLE;
            drv_lcd_fb.testdma.Init.MemInc = DMA_MINC_ENABLE;
            drv_lcd_fb.testdma.Init.Mode               = DMA_NORMAL;
            drv_lcd_fb.testdma.Init.Priority           = DMA_PRIORITY_HIGH;
            drv_lcd_fb.testdma.Init.PeriphInc = DMA_PINC_ENABLE; //src
            drv_lcd_fb.testdma.Init.MemInc = DMA_MINC_ENABLE;    //dst

            if (0 == ((src_line_addr | dst_line_addr | len) & 3)) //Word aligned
            {
                drv_lcd_fb.testdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
                drv_lcd_fb.testdma.Init.MemDataAlignment   = DMA_MDATAALIGN_WORD;
                counts = len >> 2;
            }
            else if (0 == ((src_line_addr | dst_line_addr | len) & 1)) //Half word aligned
            {
                drv_lcd_fb.testdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
                drv_lcd_fb.testdma.Init.MemDataAlignment   = DMA_MDATAALIGN_HALFWORD;
                counts = len >> 1;
            }
            else
            {
                drv_lcd_fb.testdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
                drv_lcd_fb.testdma.Init.MemDataAlignment   = DMA_MDATAALIGN_BYTE;
                counts = len;
            }

            HAL_DMA_Init(&drv_lcd_fb.testdma);

            /* NVIC configuration for DMA transfer complete interrupt */
            HAL_NVIC_SetPriority(GP_DMA_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(GP_DMA_IRQn);

            HAL_DMA_RegisterCallback(&drv_lcd_fb.testdma, HAL_DMA_XFER_ERROR_CB_ID, (void (*)(struct __DMA_HandleTypeDef *))write_fb_err_cb);

            drv_lcd_fb.src = src_line_addr;
            drv_lcd_fb.dst = dst_line_addr;
            drv_lcd_fb.left_counts = counts;
            drv_lcd_fb.dma_cb = cb;
            DMA_reload();

#ifdef DRV_LCD_FB_STATISTICS
            drv_lcd_fb.gpdma_copy_cnt++;
#endif /* DRV_LCD_FB_STATISTICS */

        }
#endif /* ENABLE_GP_DMA_COPY */
#ifdef ENABLE_AES_COPY
        else if (use_aes)
        {
            RT_ASSERT(len > 16);
            uint32_t aes_copy_bytes = len & (0xFFFFFFF0);
            uint16_t memcopy_bytes = len - aes_copy_bytes;

            drv_lcd_fb.src = src_line_addr;
            drv_lcd_fb.dst = dst_line_addr;
            drv_lcd_fb.dma_cb = cb;


            AES_IOTypeDef io_data;
            io_data.in_data = (uint8_t *)src_line_addr;
            io_data.out_data = (uint8_t *)dst_line_addr;
            io_data.size = aes_copy_bytes;
            err = drv_aes_copy_async(&io_data, AES_CopyCb);
            RT_ASSERT(RT_EOK == err);

            memcpy((uint8_t *)(src_line_addr + aes_copy_bytes), (uint8_t *)(dst_line_addr + aes_copy_bytes), memcopy_bytes);

#ifdef DRV_LCD_FB_STATISTICS
            drv_lcd_fb.aes_copy_cnt++;
#endif /* DRV_LCD_FB_STATISTICS */

        }
#endif /* ENABLE_AES_COPY */
        else
        {
            EXT_DMA_CmprTypeDef cmpr;
            uint32_t copy_words;

            //Copy the pixels to compressed buffer
            cmpr.cmpr_rate = drv_lcd_fb.fb.cmpr_rate;
            cmpr.cmpr_en   = (drv_lcd_fb.fb.cmpr_rate > 0);

            cmpr.col_num = width;
            cmpr.row_num = height;
            cmpr.src_format = (2 == bytes_per_pixel) ? EXTDMA_CMPRCR_SRCFMT_RGB565 : EXTDMA_CMPRCR_SRCFMT_RGB888;
            copy_words = RT_ALIGN(len, 4) / 4;

            err = EXT_DMA_ConfigCmpr(1, 1, &cmpr);
            RT_ASSERT(RT_EOK == err);

            EXT_DMA_Register_Callback(EXT_DMA_XFER_CPLT_CB_ID, cb);
            EXT_DMA_Register_Callback(EXT_DMA_XFER_ERROR_CB_ID, write_fb_err_cb);

            err = EXT_DMA_START_ASYNC(src_line_addr, dst_line_addr,  copy_words);
            RT_ASSERT(RT_EOK == err);

#ifdef DRV_LCD_FB_STATISTICS
            drv_lcd_fb.extdma_copy_cnt++;
#endif /* DRV_LCD_FB_STATISTICS */

        }

    }
    else
    {
#ifdef BSP_USING_EPIC
        EPIC_AreaTypeDef epic_src_area, epic_dst_area, epic_copy_area;
        uint32_t epic_cf;

        LCD_area_to_EPIC_area(src_area, &epic_src_area);
        LCD_area_to_EPIC_area(dst_area, &epic_dst_area);
        LCD_area_to_EPIC_area(clip_area, &epic_copy_area);
        switch (drv_lcd_fb.fb.format)
        {
        case RTGRAPHIC_PIXEL_FORMAT_RGB565:
            epic_cf = EPIC_INPUT_RGB565;
            break;
        case RTGRAPHIC_PIXEL_FORMAT_RGB888:
            epic_cf = EPIC_INPUT_RGB888;
            break;

        default:
            RT_ASSERT(0);
            break;
        }

        if (drv_lcd_fb.fb.cmpr_rate)
        {
            LOG_E("EPIC not support copy compressed buffer");
            RT_ASSERT(0);
        }

        err = drv_epic_copy(src, drv_lcd_fb.fb.p_data,
                            &epic_src_area, &epic_dst_area,
                            &epic_copy_area, epic_cf, epic_cf,
                            (drv_epic_cplt_cbk)cb);
        RT_ASSERT(RT_EOK == err);
#ifdef DRV_LCD_FB_STATISTICS
        drv_lcd_fb.epic_copy_cnt++;
#endif /* DRV_LCD_FB_STATISTICS */

#else
        RT_ASSERT(0);//Copy partial FB with software is not supported now
#endif /* BSP_USING_EPIC */

    }


    return err;
}



uint32_t drv_lcd_fb_init(const char *lcd_dev_name)
{
    rt_err_t err;
    memset(&drv_lcd_fb, 0, sizeof(drv_lcd_fb));

    drv_lcd_fb.p_lcd_dev = rt_device_find(lcd_dev_name);
    if (!drv_lcd_fb.p_lcd_dev)
    {
        LOG_E("Can't found %s", lcd_dev_name);
        return RT_EEMPTY;
    }
    err = rt_device_open(drv_lcd_fb.p_lcd_dev, RT_DEVICE_OFLAG_RDWR);

    if ((RT_EOK == err) || (-RT_EBUSY == err))
    {
        rt_device_control(drv_lcd_fb.p_lcd_dev, RTGRAPHIC_CTRL_GET_INFO, &drv_lcd_fb.lcd_info);
        uint16_t interval_lines;
        interval_lines = drv_lcd_fb.lcd_info.height >> 3;
        if (interval_lines < 1) interval_lines = 1;
        rt_device_control(drv_lcd_fb.p_lcd_dev, RTGRAPHIC_CTRL_IRQ_INTERVAL_LINE, &interval_lines);
    }

    err = rt_event_init(&drv_lcd_fb.event, "fb_event", RT_IPC_FLAG_FIFO);
    RT_ASSERT(err == RT_EOK);
    err = rt_event_send(&drv_lcd_fb.event, EVENT_ALL_DONE);
    RT_ASSERT(err == RT_EOK);

    drv_lcd_fb.dma_faster_than_lcdc = 1; //Assume that DMA copy always fater than LCDC
    return RT_EOK;
}



uint32_t drv_lcd_fb_set(lcd_fb_desc_t *fb_desc)
{
    rt_base_t level;
    bool new_fb = false;

    level = rt_hw_interrupt_disable();
    if (drv_lcd_fb.fb.p_data != fb_desc->p_data) //Using a new FB
    {
        memcpy(&drv_lcd_fb.fb, fb_desc, sizeof(lcd_fb_desc_t));
        memcpy(&drv_lcd_fb.window, &invalid_area, sizeof(LCD_AreaDef));
        drv_lcd_fb.scheme6_y0 = INT32_MIN;
        drv_lcd_fb.scheme6_valid_y1 = drv_lcd_fb.fb.area.y1;
        drv_lcd_fb.flushing_lcd = 0;
        new_fb = true;
    }
    rt_hw_interrupt_enable(level);

    if (new_fb)
    {
        LOG_D("Using a new FB=%x area:"AreaString" fmt=%d,cmpr=%d,lineBytes=%d", fb_desc->p_data,
              AreaParams(&fb_desc->area),
              fb_desc->format, fb_desc->cmpr_rate,
              fb_desc->line_bytes);
    }
    return RT_EOK;
}
uint32_t drv_lcd_fb_is_busy(void)
{
    rt_err_t err;

    err = rt_event_recv(&drv_lcd_fb.event, EVENT_WRITE_DONE,
                        RT_EVENT_FLAG_OR, 0, NULL);

    return (RT_EOK == err) ? 0 : 1;
}


rt_err_t drv_lcd_fb_wait_write_done(int32_t wait_ms)
{
    rt_err_t err;

    err = rt_event_recv(&drv_lcd_fb.event, EVENT_WRITE_DONE,
                        RT_EVENT_FLAG_OR,
                        rt_tick_from_millisecond(wait_ms), NULL);

    if (-RT_ETIMEOUT == err)
    {
        LOG_E("Wait_write_done for %d ms, timeout!!!", wait_ms);
    }
    return err;
}

rt_err_t drv_lcd_fb_get_write_area(LCD_AreaDef *write_area, int32_t wait_ms)
{
    rt_err_t err = RT_EOK;
    //Wait fb writebale
    LOG_D("Try get write area, expect:"AreaString, AreaParams(write_area));
    while (drv_lcd_fb.scheme6_valid_y1 < write_area->y0)
    {
        err = rt_event_recv(&drv_lcd_fb.event, EVENT_LINE_VALID | EVENT_FLUSH_FB_DONE,
                            RT_EVENT_FLAG_OR,
                            rt_tick_from_millisecond(wait_ms), NULL);

        if (RT_EOK != err) break; //Overwrite anyway
        else if (drv_lcd_fb.event.set & EVENT_FLUSH_FB_DONE) break; //Whole FB is avaiable.
    }

    if (RT_EOK == err) write_area->y1 = MIN(drv_lcd_fb.scheme6_valid_y1, write_area->y1);

    LOG_D("Got write area:"AreaString, AreaParams(write_area));
    DRV_LCD_FB_ASSERT(is_area_valid(write_area));

    return err;
}

rt_err_t drv_lcd_fb_write_send(LCD_AreaDef *write_area, LCD_AreaDef *src_area, const uint8_t *src, write_fb_cbk cb, uint8_t send)
{
    LCD_AreaDef common_area;/*Relative area of FB will be flushed*/

    if (area_intersect(&common_area, write_area, src_area))
    {
        rt_err_t err;

        LOG_D("Write area:"AreaString, AreaParams(write_area));
        LOG_D("Src area:"AreaString" src=%p", AreaParams(src_area), src);
        LOG_D("Common area:"AreaString, AreaParams(&common_area));

        err = rt_event_recv(&drv_lcd_fb.event, EVENT_WRITE_DONE,
                            RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                            rt_tick_from_millisecond(FB_COPY_EXP_MS), NULL);
        DRV_LCD_FB_ASSERT(RT_EOK == err);

        wait_line_valid(&common_area, FB_COPY_EXP_MS);

        //Join to window
        if (is_area_valid(&drv_lcd_fb.window))
        {
            drv_lcd_fb.window.x0 = MIN(drv_lcd_fb.window.x0, write_area->x0);
            drv_lcd_fb.window.y0 = MIN(drv_lcd_fb.window.y0, write_area->y0);
            drv_lcd_fb.window.x1 = MAX(drv_lcd_fb.window.x1, write_area->x1);
            drv_lcd_fb.window.y1 = MAX(drv_lcd_fb.window.y1, write_area->y1);
        }
        else
        {
            //First time
            memcpy(&drv_lcd_fb.window, write_area, sizeof(LCD_AreaDef));
        }
        LOG_D("Total window:"AreaString, AreaParams(&drv_lcd_fb.window));


        if (send)
        {
            //Wait last flushing done.
            err = rt_event_recv(&drv_lcd_fb.event, EVENT_FLUSH_FB_DONE,
                                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                                rt_tick_from_millisecond(FB_FLUSH_EXP_MS), NULL);
            DRV_LCD_FB_ASSERT(RT_EOK == err);
            DRV_LCD_FB_ASSERT(0 == drv_lcd_fb.flushing_lcd);

            drv_lcd_fb.cb = cb;
            if (drv_lcd_fb.dma_faster_than_lcdc)
            {
                write_fb_async(&common_area, src_area, src, write_fb_cb_done);
                fb_flush_start();
            }
            else
            {
                write_fb_async(&common_area, src_area, src, write_fb_cb_done_send);
            }
        }
        else
        {
            drv_lcd_fb.cb = cb;
            write_fb_async(&common_area, src_area, src, write_fb_cb_done);
        }

    }
    else
    {
        if (cb) cb(&drv_lcd_fb.fb);
    }

    return RT_EOK;
}

#endif /* BSP_USING_LCD_FRAMEBUFFER */
