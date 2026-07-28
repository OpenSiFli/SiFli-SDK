/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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
#ifdef HAL_EPICTL_ENABLED
    #include "drv_epictl.h"
#endif /*HAL_EPICTL_ENABLED*/
#ifdef BSP_USING_LCD_FRAMEBUFFER

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
    EVENT_FB0_LINE_VALID     = (1 << 0),
    EVENT_FB0_FLUSH_DONE     = (1 << 1),
    EVENT_FB1_LINE_VALID     = (1 << 2),
    EVENT_FB1_FLUSH_DONE     = (1 << 3),
    EVENT_WRITE_DONE         = (1 << 4)
};
#define EVENT_ALL_DONE (EVENT_FB0_LINE_VALID|EVENT_FB1_LINE_VALID|EVENT_FB0_FLUSH_DONE|EVENT_FB1_FLUSH_DONE|EVENT_WRITE_DONE)

typedef void (*dma_write_cbk)(void);

typedef struct
{
    lcd_fb_desc_t  fb;     /*Framebuffer description*/
    LCD_AreaDef fb_clip;    /*LCD recieve area, origin is LCD's TL*/
    uint8_t  fb_flushing_lcd;
    uint8_t  ready;   /*The FB is ready to flush LCD*/
    int32_t fb_valid_y1; /*Writeable lines are [0 ~ fb_height-1], and -1 means no writeable line.*/
    int32_t fb_flush_start_y; /*The line ,should be in [0 ~ fb_height-1],where the LCDC flushed from*/
    uint32_t flush_start_tick;       /*Last aysnc flush LCD start tick*/
    uint32_t flush_end_tick;         /*Last aysnc flush LCD end tick*/
} LCD_FBTypeDef;

typedef struct
{
    rt_device_t p_lcd_dev;
    struct rt_device_graphic_info lcd_info;

    uint16_t fb_total;
    uint16_t write_fb_idx;
    uint16_t flush_fb_idx;
    LCD_FBTypeDef fbs[2];


    struct rt_event  event;

    write_fb_cbk cb;


#ifdef CHECK_FB_WRITE_OVERFLOW
    uint8_t *overwrite_check_addr;
    uint8_t overwrite_check_golden[4];
#endif /* CHECK_FB_WRITE_OVERFLOW */

    uint32_t write_start_tick;       /*Last aysnc write framebuffer start tick*/
    uint32_t write_end_tick;         /*Last aysnc write framebuffer end tick*/

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

} DRV_LCD_FBTypeDef;



//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

static uint8_t Enable_LineCpltCbk = 0;
static DRV_LCD_FBTypeDef drv_lcd_fb;
static const LCD_AreaDef  invalid_area = {-1, -1, -2, -2};


static rt_err_t fb_flush_start(void);

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
    for (uint16_t i = 0; i < drv_lcd_fb.fb_total; i++)
    {
        LCD_FBTypeDef *p_fb = &drv_lcd_fb.fbs[i];

        LOG_E("---fbs[%d]:", i);
        LOG_E("Fb=%x area:"AreaString" fmt=%d,cmpr=%d,lineBytes=%d", p_fb->fb.p_data,
              AreaParams(&p_fb->fb.area),
              p_fb->fb.format, p_fb->fb.cmpr_rate,
              p_fb->fb.line_bytes);

        LOG_E("ready=%d, fb_flushing_lcd=%d, start_y=%d, valid_y=%d", p_fb->ready, p_fb->fb_flushing_lcd,
              p_fb->fb_flush_start_y,
              p_fb->fb_valid_y1);
        LOG_E("fb_clip:"AreaString, AreaParams(&p_fb->fb_clip));
        LOG_E("flush_start=%d,end=%d", p_fb->flush_start_tick, p_fb->flush_end_tick);
    }
    LOG_E("----");
    LOG_E("write_idx=%d,flush_idx=%d,total=%d", drv_lcd_fb.write_fb_idx, drv_lcd_fb.flush_fb_idx, drv_lcd_fb.fb_total);
    LOG_E("flush_req=%d,rsp=%d, write_req=%d,rsp=%d",
          drv_lcd_fb.dbg_flush_req, drv_lcd_fb.dbg_flush_rsp,
          drv_lcd_fb.dbg_write_req, drv_lcd_fb.dbg_write_rsp);
    LOG_E("event=%x detail:", drv_lcd_fb.event.set);
#define EVENT_LOG(e) LOG_E(#e "=%d", (0 != (drv_lcd_fb.event.set&(e))))

    EVENT_LOG(EVENT_FB0_LINE_VALID);
    EVENT_LOG(EVENT_FB0_FLUSH_DONE);
    EVENT_LOG(EVENT_FB1_LINE_VALID);
    EVENT_LOG(EVENT_FB1_FLUSH_DONE);
    EVENT_LOG(EVENT_WRITE_DONE);

}
#define DRV_LCD_FB_ASSERT(expr) do{if(!(expr)){err_debug();RT_ASSERT(0);}}while(0)


static void set_valid_y(int32_t y)
{
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    drv_lcd_fb.fbs[drv_lcd_fb.flush_fb_idx].fb_valid_y1 = y;
    rt_err_t err;
    err = rt_event_send(&drv_lcd_fb.event,
                        (0 == drv_lcd_fb.flush_fb_idx) ? EVENT_FB0_LINE_VALID : EVENT_FB1_LINE_VALID);
    RT_ASSERT(RT_EOK == err);
    rt_hw_interrupt_enable(level);

    //LOG_D("drv_lcd_fb.fb_valid_y1 %d", y);
}

static rt_err_t wait_line_valid(LCD_AreaDef *write_area, int32_t wait_ms)
{
    rt_err_t err = RT_EOK;

    uint16_t wait_fb_idx = drv_lcd_fb.write_fb_idx;
    uint32_t line_event = (0 == wait_fb_idx) ? EVENT_FB0_LINE_VALID : EVENT_FB1_LINE_VALID;
    LCD_FBTypeDef *p_fb = &drv_lcd_fb.fbs[wait_fb_idx];

    //Wait fb writebale
    while (p_fb->fb.area.y0 + p_fb->fb_valid_y1 < write_area->y1)
    {
        err = rt_event_recv(&drv_lcd_fb.event, line_event,
                            RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                            rt_tick_from_millisecond(wait_ms), NULL);

        if (RT_EOK != err) break; //Overwrite anyway
    }

    return err;
}

void HAL_LCDC_SendLineCpltCbk(LCDC_HandleTypeDef *lcdc, uint32_t line)
{
    /*
        Since rt_device 'lcd' will invoked by others, and 'HAL_LCDC_SendLineCpltCbk' will be invoke too,
        so skip it.
    */

    if (Enable_LineCpltCbk)
    {
        LCD_FBTypeDef *p_fb = &drv_lcd_fb.fbs[drv_lcd_fb.flush_fb_idx];
        if (p_fb->fb_flushing_lcd)
        {
            RT_ASSERT(p_fb->fb_flush_start_y != INT32_MIN);
            //LOG_D("SendLineCpltCbk %d \r\n", p_fb->fb_flush_start_y + line);
            set_valid_y(p_fb->fb_flush_start_y + line);
        }
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

    drv_lcd_fb.dbg_flush_rsp++;

#ifdef PKG_USING_SYSTEMVIEW
    SystemView_mark_stop(FLUSH_LCD_SYSTEMVIEW_MARK_ID);
#endif /* PKG_USING_SYSTEMVIEW */

    rt_base_t level = rt_hw_interrupt_disable();

    LCD_FBTypeDef *p_fb = &drv_lcd_fb.fbs[drv_lcd_fb.flush_fb_idx];

    DRV_LCD_FB_ASSERT(1 == p_fb->ready);
    DRV_LCD_FB_ASSERT(1 == p_fb->fb_flushing_lcd);
    p_fb->ready = 0;
    p_fb->fb_flushing_lcd = 0;
    p_fb->fb_flush_start_y = INT32_MIN;
    set_valid_y(p_fb->fb.area.y1 - p_fb->fb.area.y0);
    Enable_LineCpltCbk = 0;

    err = rt_event_send(&drv_lcd_fb.event,
                        (0 == drv_lcd_fb.flush_fb_idx) ? EVENT_FB0_FLUSH_DONE : EVENT_FB1_FLUSH_DONE);

    rt_hw_interrupt_enable(level);

    p_fb->flush_end_tick = rt_tick_get();
    LOG_D("fb_flush_done %p, cost=%d ticks", buffer, p_fb->flush_end_tick - p_fb->flush_start_tick);
    LOG_D("fb_flush_done event=%x", drv_lcd_fb.event.set);

    //Flush next one if it's ready.
    fb_flush_start();
    return err;
}

static rt_err_t fb_flush_start(void)
{
    rt_err_t err;
    rt_device_t p_lcd_dev = drv_lcd_fb.p_lcd_dev;
    LCD_AreaDef common_area;/*Relative area of FB will be flushed*/
    lcd_flush_info_t flush_info;


    LOG_D("fb_flush_start");

    rt_base_t level = rt_hw_interrupt_disable();
    if (1 == Enable_LineCpltCbk) //Previous flush is not done.
    {
        rt_hw_interrupt_enable(level);
        return RT_EBUSY;
    }

    //Find the valid framebuffer
    LCD_FBTypeDef *p_fb = &drv_lcd_fb.fbs[drv_lcd_fb.flush_fb_idx];
    if (0 == p_fb->ready)
    {
        //Flush next one.
        drv_lcd_fb.flush_fb_idx = (drv_lcd_fb.flush_fb_idx + 1) % drv_lcd_fb.fb_total;
        p_fb = &drv_lcd_fb.fbs[drv_lcd_fb.flush_fb_idx];

        if (0 == p_fb->ready)
        {
            rt_hw_interrupt_enable(level);
            LOG_D("Both framebuffers are not ready");
            return RT_EEMPTY;
        }
    }


    //Setup line interrupt handler variables
    Enable_LineCpltCbk = 1;
    DRV_LCD_FB_ASSERT(area_intersect(&common_area, &p_fb->fb_clip, &p_fb->fb.area));
    set_valid_y(common_area.y0 - p_fb->fb.area.y0 - 1);
    p_fb->fb_flush_start_y = common_area.y0 - p_fb->fb.area.y0;
    p_fb->fb_flushing_lcd = 1;


    //Setup flush info
    flush_info.cmpr_rate = p_fb->fb.cmpr_rate;
    flush_info.pixel      = p_fb->fb.p_data;
    flush_info.color_format    = p_fb->fb.format;
    lcd_area_copy(&flush_info.window, &p_fb->fb_clip);
    lcd_area_copy(&flush_info.pixel_area, &p_fb->fb.area);

    //Prepare an clean 'fb_clip' for new frame.
    lcd_area_copy(&p_fb->fb_clip, &invalid_area);

    rt_hw_interrupt_enable(level);



    //Flush to lcd
    drv_lcd_fb.dbg_flush_req++;
    p_fb->flush_start_tick = rt_tick_get();
    rt_device_set_tx_complete(drv_lcd_fb.p_lcd_dev, fb_flush_done);
#ifdef PKG_USING_SYSTEMVIEW
    SystemView_mark_start(FLUSH_LCD_SYSTEMVIEW_MARK_ID,
                          "window:"AreaString" p_data=%p", AreaParams(&flush_info.window), flush_info.pixel);
#endif /* PKG_USING_SYSTEMVIEW */
    LOG_D("fb_flush_start idx=%d", drv_lcd_fb.flush_fb_idx);
    LOG_D("window:"AreaString" fb:"AreaString" p_data=%p",
          AreaParams(&flush_info.window), AreaParams(&flush_info.pixel_area), flush_info.pixel);

    err = rt_device_control(p_lcd_dev, SF_GRAPHIC_CTRL_LCDC_FLUSH, &flush_info);

    if (RT_EOK != err)
    {
        LOG_E("fb_flush_start err=%d", err);
        fb_flush_done(p_lcd_dev, (void *)flush_info.pixel);
    }

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

    if (cb) cb(&drv_lcd_fb.fbs[drv_lcd_fb.write_fb_idx].fb);

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
    LCD_FBTypeDef *p_fb = &drv_lcd_fb.fbs[drv_lcd_fb.write_fb_idx];

    write_fb_cb1();
    p_fb->ready = 1;
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
    uint8_t  continuous_copy = 0;
#if defined(BSP_USING_EPIC) && !defined(DRV_EPIC_NEW_API)
    uint8_t  use_epic = 1;
#else
    uint8_t  use_epic = 0;
#endif
#ifdef HAL_EPICTL_ENABLED
    uint8_t  use_epictl = 1;
#endif /*HAL_EPICTL_ENABLED*/
    LCD_FBTypeDef *p_fb = &drv_lcd_fb.fbs[drv_lcd_fb.write_fb_idx];
    LCD_AreaDef *dst_area = &p_fb->fb.area;

    //src_line_addr, dst_line_addr, len are all after clipped
    uint32_t src_line_addr, dst_line_addr, len, bytes_per_pixel;
    uint32_t width, height, src_width;


    switch (p_fb->fb.format)
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

    dst_line_addr = ((uint32_t)p_fb->fb.p_data)
                    + (clip_area->y0 - p_fb->fb.area.y0) * p_fb->fb.line_bytes;
    len = bytes_per_pixel * width * height;

    LOG_D("clip area:"AreaString" src area:"AreaString" src=%p", AreaParams(clip_area), AreaParams(src_area), src);
    LOG_D("src_line_addr=0x%x dst_line_addr=0x%x len=0x%x", src_line_addr, dst_line_addr, len);

    if (((clip_area->x0 == src_area->x0) && (dst_area->x0 == src_area->x0)
            && (clip_area->x1 == src_area->x1) && (dst_area->x1 == src_area->x1)))
    {
        //Continuous buffer copy
        continuous_copy = 1;
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

    if (p_fb->fb.cmpr_rate != 0)
    {
        use_gp_dma = 0; //DMAs can't support compressed buf
        use_aes = 0;
        use_epic = 0;
        RT_ASSERT(1 == use_extdma);
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
    drv_lcd_fb.overwrite_check_addr = (uint8_t *)(p_fb->fb.p_data +
                                      p_fb->fb.line_bytes * (p_fb->fb.area.y1 - p_fb->fb.area.y0 + 1));
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


    if (src_line_addr == dst_line_addr)
    {
        LOG_D("src_line_addr==dst_line_addr skip.");
        cb();
        err = RT_EOK;
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
#ifdef HAL_EXTDMA_MODULE_ENABLED
    else if (use_extdma)
    {
        EXT_DMA_CmprTypeDef cmpr;
        uint32_t extdma_copy_bytes, memcopy_bytes, copy_words;
        RT_ASSERT(len > 4);

        if (p_fb->fb.cmpr_rate > 0)
        {
            //Use EXT-DMA copy only if compression is enabled.
            extdma_copy_bytes = RT_ALIGN(len, 4);
            memcopy_bytes = 0;
        }
        else
        {
            extdma_copy_bytes = len & (0xFFFFFFFC);
            memcopy_bytes = len - extdma_copy_bytes;
        }
        copy_words = extdma_copy_bytes >> 2;
        //Copy the pixels to compressed buffer
        cmpr.cmpr_rate = p_fb->fb.cmpr_rate;
        cmpr.cmpr_en   = (p_fb->fb.cmpr_rate > 0);
        cmpr.col_num = width;
        cmpr.row_num = height;
        cmpr.src_format = (2 == bytes_per_pixel) ? EXTDMA_CMPRCR_SRCFMT_RGB565 : EXTDMA_CMPRCR_SRCFMT_RGB888;
        err = EXT_DMA_ConfigCmpr(1, 1, &cmpr);
        RT_ASSERT(RT_EOK == err);

        EXT_DMA_Register_Callback(EXT_DMA_XFER_CPLT_CB_ID, cb);
        EXT_DMA_Register_Callback(EXT_DMA_XFER_ERROR_CB_ID, write_fb_err_cb);

        err = EXT_DMA_START_ASYNC(src_line_addr, dst_line_addr,  copy_words);
        RT_ASSERT(RT_EOK == err);

        if (memcopy_bytes > 0)
        {
            memcpy((uint8_t *)(dst_line_addr + extdma_copy_bytes), (uint8_t *)(src_line_addr + extdma_copy_bytes), memcopy_bytes);
            mpu_dcache_clean((uint8_t *)(dst_line_addr + extdma_copy_bytes), memcopy_bytes);
        }

#ifdef DRV_LCD_FB_STATISTICS
        drv_lcd_fb.extdma_copy_cnt++;
#endif /* DRV_LCD_FB_STATISTICS */

    }
#endif /* HAL_EXTDMA_MODULE_ENABLED */
#ifdef HAL_EPICTL_ENABLED
    else if (use_epictl)
    {
        EPICTL_DataType cfg;
        HAL_EPICTL_DataInit(&cfg);

        uint32_t epic_cf;
        switch (p_fb->fb.format)
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
        cfg.src = (const uint8_t *) src_line_addr;
        cfg.src_color_mode = epic_cf;
        cfg.src_stride = src_width * bytes_per_pixel;

        cfg.dst = (uint8_t *) dst_line_addr;
        cfg.dst_color_mode = epic_cf;
        cfg.dst_stride = p_fb->fb.line_bytes;
        cfg.dst_compression_rate = p_fb->fb.cmpr_rate;

        cfg.transfer_width = width;
        cfg.transfer_height = height;

        err = drv_epictl_transfer(&cfg, (drv_epictl_cplt_cbk)cb);
        RT_ASSERT(RT_EOK == err);
    }
#endif /*HAL_EPICTL_ENABLED*/
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

        if (memcopy_bytes > 0)
        {
            memcpy((uint8_t *)(dst_line_addr + aes_copy_bytes), (uint8_t *)(src_line_addr + aes_copy_bytes), memcopy_bytes);
            mpu_dcache_clean((uint8_t *)(dst_line_addr + aes_copy_bytes), memcopy_bytes);
        }
#ifdef DRV_LCD_FB_STATISTICS
        drv_lcd_fb.aes_copy_cnt++;
#endif /* DRV_LCD_FB_STATISTICS */

    }
#endif /* ENABLE_AES_COPY */
    else if (use_epic)
    {
#if defined(BSP_USING_EPIC) && !defined(DRV_EPIC_NEW_API)
        EPIC_AreaTypeDef epic_src_area, epic_dst_area, epic_copy_area;
        uint32_t epic_cf;

        LCD_area_to_EPIC_area(src_area, &epic_src_area);
        LCD_area_to_EPIC_area(dst_area, &epic_dst_area);
        LCD_area_to_EPIC_area(clip_area, &epic_copy_area);
        switch (p_fb->fb.format)
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

        if (p_fb->fb.cmpr_rate)
        {
            LOG_E("EPIC not support copy compressed buffer");
            RT_ASSERT(0);
        }

        err = drv_epic_copy(src, p_fb->fb.p_data,
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
    else //Software copy
    {
        if (continuous_copy)
        {
            memcpy((uint8_t *)dst_line_addr, (uint8_t *)src_line_addr, len);
            mpu_dcache_clean((uint8_t *)dst_line_addr, len);
        }
        else
        {
            RT_ASSERT(0);//Copy partial FB with software is not supported now
        }
        cb();
        err = RT_EOK;
    }



    return err;
}



uint32_t drv_lcd_fb_init(const char *lcd_dev_name)
{
    rt_err_t err;
    LOG_I("drv_lcd_fb_init");

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
        /* If the LCD interrupt interval is too long, it will lead to a lot of idle waiting.
         * Therefore, change it back to trigger an interrupt every 10 lines.*/
        interval_lines = 10;//drv_lcd_fb.lcd_info.height >> 3;
        if (interval_lines < 1) interval_lines = 1;
        rt_device_control(drv_lcd_fb.p_lcd_dev, RTGRAPHIC_CTRL_IRQ_INTERVAL_LINE, &interval_lines);
    }

    err = rt_event_init(&drv_lcd_fb.event, "fb_event", RT_IPC_FLAG_FIFO);
    RT_ASSERT(err == RT_EOK);
    err = rt_event_send(&drv_lcd_fb.event, EVENT_ALL_DONE);
    RT_ASSERT(err == RT_EOK);

    drv_lcd_fb.dma_faster_than_lcdc = 1; //Assume that DMA copy always fater than LCDC

    LOG_I("drv_lcd_fb_init done.");
    return RT_EOK;
}

uint32_t drv_lcd_fb_deinit(void)
{
    LOG_I("drv_lcd_fb_deinit");

    rt_err_t err;

    err = rt_event_recv(&drv_lcd_fb.event, EVENT_ALL_DONE,
                        RT_EVENT_FLAG_AND, FB_FLUSH_EXP_MS, NULL);
    RT_ASSERT(err == RT_EOK);
    err = rt_event_detach(&drv_lcd_fb.event);
    RT_ASSERT(err == RT_EOK);

    if (drv_lcd_fb.p_lcd_dev)
    {
        err = rt_device_close(drv_lcd_fb.p_lcd_dev);
        RT_ASSERT(err == RT_EOK);
    }
    LOG_I("drv_lcd_fb_deinit done.");

    return RT_EOK;
}


uint32_t drv_lcd_fb_set(lcd_fb_desc_t *fb_desc)
{
    rt_base_t level;
    bool new_fb = false;

    level = rt_hw_interrupt_disable();
    LCD_FBTypeDef *p_fb_curr = &drv_lcd_fb.fbs[drv_lcd_fb.write_fb_idx];

    if (p_fb_curr->fb.p_data == fb_desc->p_data)
    {
        ;
    }
    else if (2 == drv_lcd_fb.fb_total)
    {
        uint16_t next_idx = (drv_lcd_fb.write_fb_idx + 1) % drv_lcd_fb.fb_total;
        RT_ASSERT(drv_lcd_fb.fbs[next_idx].fb.p_data == fb_desc->p_data);

        drv_lcd_fb.write_fb_idx = next_idx;
    }
    else //Using a new FB
    {
        RT_ASSERT(drv_lcd_fb.fb_total < 2);
        new_fb = true;
        p_fb_curr = &drv_lcd_fb.fbs[drv_lcd_fb.fb_total];
        memcpy(&p_fb_curr->fb, fb_desc, sizeof(lcd_fb_desc_t));
        lcd_area_copy(&p_fb_curr->fb_clip, &invalid_area);
        p_fb_curr->fb_flush_start_y = INT32_MIN;
        p_fb_curr->fb_valid_y1 = fb_desc->area.y1 - fb_desc->area.y0;
        p_fb_curr->fb_flushing_lcd = 0;
        p_fb_curr->ready = 0;
        drv_lcd_fb.fb_total++;

        //Set write idx to this fb
        drv_lcd_fb.write_fb_idx = drv_lcd_fb.fb_total - 1;
    }
    rt_hw_interrupt_enable(level);

    if (new_fb)
    {
        LOG_D("Using a new FB=%x area:"AreaString" fmt=%d,cmpr=%d,lineBytes=%d", fb_desc->p_data,
              AreaParams(&fb_desc->area),
              fb_desc->format, fb_desc->cmpr_rate,
              fb_desc->line_bytes);
    }
    LOG_D("drv_lcd_fb_set write_fb_idx: %d", drv_lcd_fb.write_fb_idx);
    return RT_EOK;
}
uint32_t drv_lcd_fb_is_busy(void)
{
    rt_err_t err;

    err = rt_event_recv(&drv_lcd_fb.event, EVENT_ALL_DONE,
                        RT_EVENT_FLAG_AND, 0, NULL);

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
//Waiting for the 'p_data' buffer not been r/w anymore.
rt_err_t drv_lcd_fb_wait_all_done(uint8_t *p_data, int32_t wait_ms)
{
    rt_err_t err = RT_EOK;
    uint32_t events1, events2;
    LCD_FBTypeDef *p_fb = NULL;

    for (uint16_t i = 0; i < drv_lcd_fb.fb_total; i++)
    {
        p_fb = &drv_lcd_fb.fbs[i];
        if (p_data == p_fb->fb.p_data)
        {
            if (0 == i)
            {
                events1 = EVENT_FB0_LINE_VALID | EVENT_FB0_FLUSH_DONE;
                events2 = EVENT_FB1_FLUSH_DONE;
            }
            else
            {
                events1 = EVENT_FB1_LINE_VALID | EVENT_FB1_FLUSH_DONE;
                events2 = EVENT_FB0_FLUSH_DONE;
            }
            err = rt_event_recv(&drv_lcd_fb.event, events1,
                                RT_EVENT_FLAG_AND,
                                rt_tick_from_millisecond(wait_ms), NULL);

            if (RT_EOK != err)
            {
                //Overwrite anyway
                LOG_W("Wait fb=%x evt=%x, timeout", p_data, events1);
            }

#ifdef BSP_USING_RAMLESS_LCD
            //Make sure the RAMLESS LCD read from another buffer
            RT_ASSERT(2 == drv_lcd_fb.fb_total);
            err = rt_event_recv(&drv_lcd_fb.event, events2,
                                RT_EVENT_FLAG_OR,
                                rt_tick_from_millisecond(wait_ms), NULL);

            if (RT_EOK != err)
            {
                //Overwrite anyway
                LOG_W("Wait fb=%x evt=%x, timeout", p_data, events2);
            }
#endif /*BSP_USING_RAMLESS_LCD*/

            return RT_EOK;
        }
    }

    return RT_EINVAL;
}

rt_err_t drv_lcd_fb_get_write_area(LCD_AreaDef *write_area, int32_t wait_ms)
{
    rt_err_t err = RT_EOK;
    uint32_t events1, events2;
    LCD_FBTypeDef *p_fb = &drv_lcd_fb.fbs[drv_lcd_fb.write_fb_idx];

    if (0 == drv_lcd_fb.write_fb_idx)
    {
        events1 = EVENT_FB0_LINE_VALID | EVENT_FB0_FLUSH_DONE;
        events2 = EVENT_FB0_FLUSH_DONE;
    }
    else
    {
        events1 = EVENT_FB1_LINE_VALID | EVENT_FB1_FLUSH_DONE;
        events2 = EVENT_FB1_FLUSH_DONE;
    }

    //Wait fb writebale
    LOG_D("Try get write area, expect:"AreaString, AreaParams(write_area));
    while (p_fb->fb.area.y0 + p_fb->fb_valid_y1 < write_area->y0)
    {
        err = rt_event_recv(&drv_lcd_fb.event, events1,
                            RT_EVENT_FLAG_OR,
                            rt_tick_from_millisecond(wait_ms), NULL);

        if (RT_EOK != err) break; //Overwrite anyway
        else if (drv_lcd_fb.event.set & events2) break; //Whole FB is avaiable.
    }

    if (RT_EOK == err) write_area->y1 = MIN(p_fb->fb.area.y0 + p_fb->fb_valid_y1, write_area->y1);

    LOG_D("Got write area:"AreaString, AreaParams(write_area));
    DRV_LCD_FB_ASSERT(is_area_valid(write_area));

    return err;
}

rt_err_t drv_lcd_fb_write_send(LCD_AreaDef *write_area, LCD_AreaDef *src_area, const uint8_t *src, write_fb_cbk cb, uint8_t send)
{
    LCD_AreaDef common_area;/*Relative area of FB will be flushed*/
    LCD_FBTypeDef *p_fb = &drv_lcd_fb.fbs[drv_lcd_fb.write_fb_idx];

    if (area_intersect(&common_area, write_area, src_area))
    {
        rt_err_t err;
        uint32_t events;
        if (0 == drv_lcd_fb.write_fb_idx)
            events = EVENT_FB0_FLUSH_DONE;
        else
            events = EVENT_FB1_FLUSH_DONE;

        LOG_D("\n\nWrite fb idx: %d, send=%d", drv_lcd_fb.write_fb_idx, send);
        LOG_D("Write area:"AreaString, AreaParams(write_area));
        LOG_D("Src area:"AreaString" src=%p", AreaParams(src_area), src);
        LOG_D("Common area:"AreaString, AreaParams(&common_area));

        err = rt_event_recv(&drv_lcd_fb.event, EVENT_WRITE_DONE,
                            RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                            rt_tick_from_millisecond(FB_COPY_EXP_MS), NULL);
        DRV_LCD_FB_ASSERT(RT_EOK == err);

        wait_line_valid(&common_area, FB_COPY_EXP_MS);

        //Join to fb_clip
        rt_base_t level = rt_hw_interrupt_disable();
        if ((1 == p_fb->ready) && (0 == p_fb->fb_flushing_lcd))
        {
            /*
                 Overwrite this fb:
                 1. mark it as not ready to avoid flushing
                 2. keep previous fb_clip, so that we can flush them together
             */
            p_fb->ready = 0;
            rt_event_send(&drv_lcd_fb.event, events);
        }
        if (is_area_valid(&p_fb->fb_clip))
        {
            p_fb->fb_clip.x0 = MIN(p_fb->fb_clip.x0, write_area->x0);
            p_fb->fb_clip.y0 = MIN(p_fb->fb_clip.y0, write_area->y0);
            p_fb->fb_clip.x1 = MAX(p_fb->fb_clip.x1, write_area->x1);
            p_fb->fb_clip.y1 = MAX(p_fb->fb_clip.y1, write_area->y1);
        }
        else
        {
            //First time
            lcd_area_copy(&p_fb->fb_clip, write_area);
        }
        rt_hw_interrupt_enable(level);
        LOG_D("Total fb_clip:"AreaString, AreaParams(&p_fb->fb_clip));


        if (send)
        {
            //Wait last flushing done.
            err = rt_event_recv(&drv_lcd_fb.event, events,
                                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                                rt_tick_from_millisecond(FB_FLUSH_EXP_MS), NULL);
            DRV_LCD_FB_ASSERT(RT_EOK == err);
            DRV_LCD_FB_ASSERT(0 == p_fb->fb_flushing_lcd);

            drv_lcd_fb.cb = cb;
            if (drv_lcd_fb.dma_faster_than_lcdc)
            {
                write_fb_async(&common_area, src_area, src, write_fb_cb_done);
                p_fb->ready = 1;
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
        if (cb) cb(&p_fb->fb);
    }

    return RT_EOK;
}

void drv_lcd_fb_write_send_parallel(uint8_t enable)
{
    drv_lcd_fb.dma_faster_than_lcdc = enable;
}

#endif /* BSP_USING_LCD_FRAMEBUFFER */
