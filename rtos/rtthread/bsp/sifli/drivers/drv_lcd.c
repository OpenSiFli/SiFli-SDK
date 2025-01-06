/**
  ******************************************************************************
  * @file   drv_lcd.c
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
#include "drv_lcd.h"
#include "drv_io.h"
#include "drv_common.h"
#include "drv_lcd_private.h"
#include "drv_ext_dma.h"
#include "mem_section.h"
#include "string.h"
#ifdef HAL_EZIP_MODULE_ENABLED
    #include "drv_flash.h"
#endif
#ifdef BSP_USING_EPIC
    #include "drv_epic.h"
#endif /* BSP_USING_EPIC */
#ifdef BSP_USING_LCD_FRAMEBUFFER
    #include "drv_lcd_fb.h"
#endif /* BSP_USING_LCD_FRAMEBUFFER */


#define  DBG_LEVEL            DBG_INFO  //DBG_LOG //

#define LOG_TAG                "drv.lcd"
#include "log.h"



#if defined(SOC_BF0_HCPU)&&!defined(BSP_USE_LCDC2_ON_HPSYS)
    #define DRV_LCD_USE_LCDC1
#endif

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

#if defined(PKG_USING_LITTLEVGL2RTT) && defined(LV_FRAME_BUF_SCHEME_0)
    /*
    LVGL defined compress buffer macros
    */
    /* TODO: all frame buffer related macro put in drv_lcd.h? */
    #include "lvgl.h"

    #define COPY2COMPRESS_FB_AND_SEND

    #if LV_COLOR_DEPTH == 16
        #define COMPRESSED_FRAME_BUF_FMT  EXTDMA_CMPRCR_SRCFMT_RGB565
    #elif LV_COLOR_DEPTH == 24
        #define COMPRESSED_FRAME_BUF_FMT  EXTDMA_CMPRCR_SRCFMT_RGB888
    #else
        #define COMPRESSED_FRAME_BUF_FMT  EXTDMA_CMPRCR_SRCFMT_ARGB8888
    #endif


    #define SRC_FRAME_BUF_BPP LV_COLOR_DEPTH
    #define MAX_SRC_FRAMEBUFFER_BYTES (FB_ALIGNED_HOR_RES * LV_VER_RES_MAX * LV_COLOR_DEPTH / 8)
    #define MAX_LINEBUFFER_BYTES (FB_ALIGNED_HOR_RES * LV_COLOR_DEPTH / 8)
    #define MAX_LINE_NUM LV_VER_RES_MAX

#else

    /*
    Default compressed buffer configuration
    */
    //#define COPY2COMPRESS_FB_AND_SEND

    #define COMPRESSED_FRAME_BUF_FMT    EXTDMA_CMPRCR_SRCFMT_RGB888
    #define SRC_FRAME_BUF_BPP           24
    #define MAX_SRC_FRAMEBUFFER_BYTES   (454*454*SRC_FRAME_BUF_BPP/8)
    #define MAX_LINEBUFFER_BYTES        (454*SRC_FRAME_BUF_BPP/8)
    #define MAX_LINE_NUM                (454)



#endif /*PKG_USING_LITTLEVGL2RTT && LV_FRAME_BUF_SCHEME_0*/




#ifdef COPY2COMPRESS_FB_AND_SEND

#define COMPRESS_BUFF_MAGIC_FLAG 0x9527BEEF
#define COMPRESSED_FRAME_BUF_NUM  (2)
#if defined(BSP_USING_BOARD_FPGA_A0) || defined(BSP_USING_BOARD_SF32LB58X_FPGA) || defined(BSP_USING_BOARD_BUTTERFLITE_FPGA)
    #define FRAME_BUF_CMPR_RATE   (0)
#elif defined(BSP_USE_LCDC2_ON_HPSYS)|| !defined(LCDC_SUPPORTED_COMPRESSED_LAYER)
    #define FRAME_BUF_CMPR_RATE   (0) //Ext dma if not avaiable
#else
    #define FRAME_BUF_CMPR_RATE   (1)
#endif

#if (FRAME_BUF_CMPR_RATE == 0)
    #define COMPRESS_BUF_SIZE_IN_WORDS   RT_ALIGN(MAX_SRC_FRAMEBUFFER_BYTES, 4)/4
#elif (FRAME_BUF_CMPR_RATE == 1)
    #if SRC_FRAME_BUF_BPP == 16
        #define COMPRESS_BUF_SIZE_IN_WORDS     (RT_ALIGN((MAX_LINEBUFFER_BYTES * 3), 16) / 16 * MAX_LINE_NUM)
    #elif SRC_FRAME_BUF_BPP == 24
        #define COMPRESS_BUF_SIZE_IN_WORDS     (RT_ALIGN(MAX_LINEBUFFER_BYTES, 8) / 8 * MAX_LINE_NUM)
    #else
        #error "Fix me"
    #endif
#else
    #error "wrong compression rate"
#endif



typedef enum
{
    FRAME_BUF_EMPTY,
    FRAME_BUF_HALF_FULL,
    FRAME_BUF_FULL
} frame_buf_status_t;



typedef struct
{
    uint32_t *addr;
    uint8_t cmpr_rate;

    LCD_AreaDef window;
    LCD_AreaDef fb_rect;
    uint32_t start_tick;  //Start DMA copy (buffer allocated)
    uint32_t cp_done_tick; //DMA copy done
    uint32_t end_tick;     //Flushed to LCD (buffer free)
    uint32_t writetimes;  //Been overwrite if large than 1
    struct rt_semaphore sema; //The buf is writting or flushing
} frame_buf_t;

typedef struct
{
    frame_buf_t frame_buf[COMPRESSED_FRAME_BUF_NUM];
    struct rt_semaphore dma_sema;
    LCD_AreaDef tmp_window; //Record last 'set_window' area
    frame_buf_t *dma_copy_buf; //Current DMA copy buf
    const char *dma_copy_src_buf;  //Current DMA copy buf source
} frame_buf_ctx_t;


static frame_buf_ctx_t compress_fb_ctx;
static uint8_t cmpr_rate = FRAME_BUF_CMPR_RATE;


L2_NON_RET_BSS_SECT_BEGIN(comp_frambuf)
L2_NON_RET_BSS_SECT(comp_frambuf, ALIGN(4) static uint32_t compress_buf1[COMPRESS_BUF_SIZE_IN_WORDS + 1 /*Overflow flag*/]);
#if COMPRESSED_FRAME_BUF_NUM > 1
    L2_NON_RET_BSS_SECT(comp_frambuf, ALIGN(4) static uint32_t compress_buf2[COMPRESS_BUF_SIZE_IN_WORDS  + 1/*Overflow flag*/]);
#endif
L2_NON_RET_BSS_SECT_END

#define EventStartA(v) //rt_kprintf("EventStartA <<<<<<<<<<<<<  %d \r\n", v)
#define EventStopA(v)  //rt_kprintf("EventStopA >>>>>>>>>>>   %d \r\n", v)


static void copy2compress_fb_init(void);
static void copy2compress_fb_deinit(void);

static frame_buf_t *alloc_frame_buf(void);
static void free_frame_buf(frame_buf_t *buf);
static frame_buf_t *get_pending_frame_buf(void);
static rt_err_t flush_pending_buf(void);
static void api_compress_fb_set_window(int x0, int y0, int x1, int y1);
static frame_buf_t *copy2compress_fb_async(const char *pixels, int32_t x0, int32_t y0, int32_t x1, int32_t y1);
static void api_copy2compress_fb_and_send_sync(const char *pixels, int32_t x0, int32_t y0, int32_t x1, int32_t y1);
static void api_copy2compress_fb_and_send_async(const char *pixels, int32_t x0, int32_t y0, int32_t x1, int32_t y1);
static rt_err_t try_flush_pending_buf(void);

#endif /* COPY2COMPRESS_FB_AND_SEND */


L1_NON_RET_BSS_SECT_BEGIN(drv_lcd_stack)
ALIGN(RT_ALIGN_SIZE)
L1_NON_RET_BSS_SECT(drv_lcd_stack, static uint8_t drv_lcd_stack[2048]);
L1_NON_RET_BSS_SECT_END


static LCD_DrvTypeDef drv_lcd;

#ifdef BSP_USING_RAMLESS_LCD
    #ifdef BSP_PM_FREQ_SCALING
        #error "Not supported on ramless LCD"
    #endif
    static uint32_t *ramless_code = NULL;

    #ifdef BSP_LCDC_USING_DPI
        //We don't know upper color format, just define the maximum RGB888 format
        #define SRAM_BUF_1LINE (LCD_HOR_RES_MAX * 3)

        #define SRAM_BUF_1LINE_WORDS (HAL_ALIGN(SRAM_BUF_1LINE, 4) / 4)
        #define SRAM_BUF_MAGIC_NUM (0xabcdefaa)
        #define SRAM_BUF_TOTAL_WORDS  SRAM_BUF_1LINE_WORDS + 1 /*overwrite examination*/


        static uint32_t *sram_data0 = NULL;
        static uint32_t *sram_data1 = NULL;
    #endif /* BSP_LCDC_USING_DPI */

    #if defined(SF32LB52X)
        #define IS_DMA_FRIENDLY_SRAM(addr)    HCPU_IS_SRAM_ADDR(addr)
    #else
        #define IS_DMA_FRIENDLY_SRAM(addr)    ((((addr) >= HPSYS_RETM_BASE) && ((addr) < HPSYS_RETM_END)) ? false : HCPU_IS_SRAM_ADDR(addr))
    #endif
    #define IS_DMA_FRIENDLY_SRAM_RANGE(p, len)        (IS_DMA_FRIENDLY_SRAM((uint32_t)p) && IS_DMA_FRIENDLY_SRAM(((uint32_t)p) + (len)))


#endif /* BSP_USING_RAMLESS_LCD */


RETM_BSS_SECT_BEGIN(lcd_idle_status)
static uint8_t lcd_idle_status;
RETM_BSS_SECT_END



static void lcd_display_on(void);
static void lcd_display_off(void);
static rt_err_t lcd_set_brightness(uint8_t bright);
static void lcd_driver_print_error_info(void);
static void lcd_driver_error_handle(void);
//static void async_send_timeout_handler(void *parameter);
static void aysnc_send_cmplt_cbk(LCD_DrvTypeDef *p_drvlcd, const uint8_t *buffer);
static void lcd_idlemode_on(void);
static void lcd_idlemode_off(void);
static void lcd_rotate(uint16_t degree);
static void lcd_task(void *param);


#ifdef BSP_USING_BOARD_MPW_EVB

    #define  LCD_BACKLIGHT_POWER_PIN    1

#endif


#define WAIT_SEMA_TIMEOUT()  do{\
    lcd_driver_print_error_info(); \
    lcd_driver_error_handle();\
    LOG_E("lcd_timeout_error %s, %d",__FUNCTION__, __LINE__);\
    RT_ASSERT(0);\
}while(0)

/*
    Not found right LCD driver or Flushing timeout occurred.
*/
#define IS_DRV_LCD_ERROR() ((LCD_STATUS_DISPLAY_TIMEOUT == drv_lcd.status) || (NULL == drv_lcd.p_drv_ops))
/*For drv_lcd must call 'RTGRAPHIC_CTRL_POWERON' after 'RT_DEVICE_CTRL_SUSPEND',
    Or exit idle mode  to resume LCDC
*/
#define IS_LCDC_OFF()  (LCD_STATUS_DISPLAY_OFF == drv_lcd.status)

static const char *lcd_s_name(uint32_t s)
{
    switch (s)
    {
    case LCD_STATUS_NONE:
        return "NONE";
    case LCD_STATUS_OPENING:
        return "OPENING";
    case LCD_STATUS_NOT_FIND_LCD:
        return "NOT_FIND";
    case LCD_STATUS_INITIALIZED:
        return "INITIALIZED";
    case LCD_STATUS_DISPLAY_ON:
        return "ON";
    case LCD_STATUS_DISPLAY_OFF:
        return "OFF";
    case LCD_STATUS_DISPLAY_TIMEOUT:
        return "TIMEOUT";
    case LCD_STATUS_IDLE_MODE:
        return "IDLEMODE";
    default:
        return "Unknow";
    }
}

static void set_drv_lcd_state(uint32_t new_state)
{
    uint32_t old_state;
    rt_base_t mask = rt_hw_interrupt_disable();
    if (new_state != drv_lcd.status)
    {
        //Keep timeout error not changed
        RT_ASSERT(drv_lcd.status != LCD_STATUS_DISPLAY_TIMEOUT);

        old_state = drv_lcd.status;
        drv_lcd.status = new_state;
    }
    else
    {
        old_state = new_state;
    }
    rt_hw_interrupt_enable(mask);


    if (old_state != new_state)
    {
        LOG_I("[%s] -> [%s]", lcd_s_name(old_state), lcd_s_name(new_state));
    }
}

static char *lcd_msg_to_name(LCD_MsgIdDef msg)
{
#define EVENT_TO_NAME_CASE(msg) case msg: return #msg
    switch (msg)
    {
        EVENT_TO_NAME_CASE(LCD_MSG_OPEN);
        EVENT_TO_NAME_CASE(LCD_MSG_CLOSE);
        EVENT_TO_NAME_CASE(LCD_MSG_SET_MODE);
        EVENT_TO_NAME_CASE(LCD_MSG_CTRL_SET_LCD_PRESENT);
        EVENT_TO_NAME_CASE(LCD_MSG_CTRL_ASSERT_IF_DRAWTIMEOUT);
        EVENT_TO_NAME_CASE(LCD_MSG_POWER_ON);
        EVENT_TO_NAME_CASE(LCD_MSG_POWER_OFF);
        EVENT_TO_NAME_CASE(LCD_MSG_DRAW_RECT_ASYNC);
        EVENT_TO_NAME_CASE(LCD_MSG_DRAW_COMP_RECT_ASYNC);
        EVENT_TO_NAME_CASE(LCD_MSG_SET_BRIGHTNESS);

        EVENT_TO_NAME_CASE(LCD_MSG_DRAW_RECT);
        EVENT_TO_NAME_CASE(LCD_MSG_SET_WINDOW);
        EVENT_TO_NAME_CASE(LCD_MSG_SET_PIXEL);
        EVENT_TO_NAME_CASE(LCD_MSG_GET_PIXEL);


    default:
        return "UNKNOW";
    }
}

#ifdef QAD_SPI_USE_GPIO_CS


void gpio_cs_init(void)
{
    rt_pin_mode(QAD_SPI_USE_GPIO_CS, PIN_MODE_OUTPUT);
    rt_pin_write(QAD_SPI_USE_GPIO_CS, 1);
}




void gpio_cs_enable(void)
{
    rt_pin_write(QAD_SPI_USE_GPIO_CS, 0);
}

void gpio_cs_disable(void)
{
    rt_pin_write(QAD_SPI_USE_GPIO_CS, 1);
}
#endif /* QAD_SPI_USE_GPIO_CS */


LCDC_HandleTypeDef *get_drv_lcd_handler(void)
{
    return &drv_lcd.hlcdc;
}

static rt_err_t api_lock(LCD_DrvTypeDef *p_drvlcd)
{
    rt_err_t err;
#define MAX_LCD_API_TIME  (10*1000)  /*10 seconds should be enough for LCD initialization.*/
    err = rt_sem_take(&(p_drvlcd->sem), rt_tick_from_millisecond(MAX_LCD_API_TIME));


    return err;
}

static void api_unlock(LCD_DrvTypeDef *p_drvlcd)
{
    rt_err_t err;

    RT_ASSERT(p_drvlcd);
    RT_ASSERT(0 == p_drvlcd->sem.value);
    err = rt_sem_release(&(p_drvlcd->sem));
    RT_ASSERT(RT_EOK == err);
}

static void print_all_msgs(rt_mq_t mq)
{
    struct rt_mq_message
    {
        struct rt_mq_message *next;
    };

    struct rt_mq_message *q_ptr;
    /* get message from queue */
    q_ptr = (struct rt_mq_message *)mq->msg_queue_head;

    for (uint32_t i = 0; i < mq->entry; i++)
    {
        if (q_ptr)
        {
            LCD_MsgTypeDef *msg;
            msg = (LCD_MsgTypeDef *)(q_ptr + 1);
            q_ptr = q_ptr->next;

            LOG_E("[%d] id:%d,tick:%d,drv:%x", i, msg->id, msg->tick, msg->driver);
        }
        else
        {
            break;
        }
    }
}

static rt_err_t send_msg_to_lcd_task(LCD_MsgTypeDef *msg)
{
    rt_err_t err;
    static uint32_t tick_cnt = 0;

    LCD_DrvTypeDef *p_drvlcd = msg->driver;
    msg->tick = tick_cnt++;
    LOG_D("send msg%x [%s] to lcd.", msg->tick, lcd_msg_to_name(msg->id));
    err = rt_mq_send(p_drvlcd->mq, msg, sizeof(*msg));

    if (err != RT_EOK)
    {
        LOG_E("send msg[%d] to lcd err:%d, %d msgs.", msg->id, err, p_drvlcd->mq->entry);
        print_all_msgs(p_drvlcd->mq);
        RT_ASSERT(0);
    }
    else
    {
        if (IS_SYNC_MSG_ID(msg->id))
        {
            //Wait msg been execution for synchronize msg
            err = rt_sem_take(&(p_drvlcd->sync_msg_sem), RT_WAITING_FOREVER);
            RT_ASSERT(RT_EOK == err);
            LOG_D("msg%x taken sync_msg_sem.", msg->tick);
        }
        else
        {
            ;
        }
    }


    return err;
}


static void enable_low_power(LCD_DrvTypeDef *p_drvlcd)
{
    HAL_StatusTypeDef err;

    LOG_D("LCDC Enter lowpower mode");

    if (p_drvlcd->auto_lowpower)
    {
        err = HAL_LCDC_Enter_LP(&p_drvlcd->hlcdc);
        if (HAL_OK != err)
        {
            LOG_E("Enter lowpower mode err %d", err);
        }
    }
    LOG_D("LCDC Enter lowpower mode done.");
}


static void disable_low_power(LCD_DrvTypeDef *p_drvlcd)
{
    HAL_StatusTypeDef err;

    LOG_D("LCDC Exit lowpower mode");
    err = HAL_LCDC_Exit_LP(&p_drvlcd->hlcdc);
    if (HAL_OK != err)
    {
        LOG_E("Exit lowpower mode err %d", err);
    }
}




#ifdef BSP_USING_LCDC
static void SendLayerDataCpltCbk(LCDC_HandleTypeDef *lcdc)
{
    LCD_DrvTypeDef *p_drvlcd = rt_container_of(lcdc, LCD_DrvTypeDef, hlcdc);

    p_drvlcd->debug_cnt2++;

    if (lcdc->XferCpltCallback != NULL)
    {
        rt_err_t err;
        lcdc->XferCpltCallback = NULL;


        err = rt_sem_release(&p_drvlcd->draw_sem);
        RT_ASSERT(RT_EOK == err);
    }

}

#ifdef LCDC_SUPPORT_LINE_DONE_IRQ
static void SendLineDoneCpltCbk(LCDC_HandleTypeDef *lcdc, uint32_t lines)
{
#if 0
    uint32_t roi_top = GET_REG_VAL(lcdc->Instance->CANVAS_TL_POS,  LCD_IF_CANVAS_TL_POS_Y0_Msk, LCD_IF_CANVAS_TL_POS_Y0_Pos);
    uint32_t roi_bottom = GET_REG_VAL(lcdc->Instance->CANVAS_BR_POS,  LCD_IF_CANVAS_BR_POS_Y1_Msk, LCD_IF_CANVAS_BR_POS_Y1_Pos);

#ifndef SF32LB55X
    uint32_t v = GET_REG_VAL(lcdc->Instance->SETTING, LCD_IF_SETTING_LINE_DONE_NUM_Msk, LCD_IF_SETTING_LINE_DONE_NUM_Pos);
    uint32_t cur_rio_line = GET_REG_VAL(lcdc->Instance->CANVAS_STAT0, LCD_IF_CANVAS_STAT0_Y_COR_Msk, LCD_IF_CANVAS_STAT0_Y_COR_Pos);
#else
    uint32_t v = roi_top + lines;
    uint32_t cur_rio_line = hwp_ptc1->MEM4;
#endif /* SF32LB55X */

    LOG_D("rio[%d~%d], irq_line=%d, cur_rio_line=%d", roi_top, roi_bottom, v, cur_rio_line);

#endif
}
#endif /* LCDC_SUPPORT_LINE_DONE_IRQ */



static void SendLayerDataErrCbk(LCDC_HandleTypeDef *lcdc)
{
    LOG_E("SendLayerData Error");
    lcd_driver_print_error_info();
}



#endif


static lcd_drv_desc_t *find_right_driver(void)
{

    lcd_drv_desc_t *table_begin = NULL;
    lcd_drv_desc_t *table_end = NULL;
    lcd_drv_desc_t *p_drv_desc = NULL;

#if defined(__CC_ARM) || (defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))                                 /* ARM C Compiler */
    extern const int LcdDriverDescTab$$Base;
    extern const int LcdDriverDescTab$$Limit;
    table_begin = (lcd_drv_desc_t *) &LcdDriverDescTab$$Base;
    table_end = (lcd_drv_desc_t *)   &LcdDriverDescTab$$Limit;
#elif defined (__ICCARM__) || defined(__ICCRX__)      /* for IAR Compiler */
#error "tobe contribute"
#elif defined (__GNUC__)                              /* for GCC Compiler */
    extern const int LcdDriverDescTab_start;
    extern const int LcdDriverDescTab_end;
    table_begin = (lcd_drv_desc_t *)&LcdDriverDescTab_start;
    table_end = (lcd_drv_desc_t *) &LcdDriverDescTab_end;
#endif /* defined(__CC_ARM) */

    if ((NULL == table_begin) || (NULL == table_end) || (table_begin == table_end))
    {
        LOG_W("No LCD driver registered!");
        return NULL;
    }

    LOG_I("Try registered LCD driver...");
    if (drv_lcd.force_lcd_missing)
    {
        LOG_I("Froce LCD missing!");
    }
    else
    {

#ifndef LCD_MISSING
        for (p_drv_desc = table_begin; p_drv_desc < table_end; p_drv_desc++)
        {
            if ((p_drv_desc->p_ops != NULL) && (p_drv_desc->p_init_cfg != NULL))
            {
                if (p_drv_desc->p_ops->ReadID != NULL)
                {
                    uint32_t id;

                    if (p_drv_desc->p_ops->Init != NULL)
                        p_drv_desc->p_ops->Init(&drv_lcd.hlcdc);

                    id = p_drv_desc->p_ops->ReadID(&drv_lcd.hlcdc);

                    if (p_drv_desc->id == id)
                    {
                        LOG_I("Found lcd %s id:%xh", p_drv_desc->name, id);
                        return p_drv_desc;
                    }
                    else
                    {
                        LOG_I("Try lcd %s, read id:%xh, expect:%xh", p_drv_desc->name, id, p_drv_desc->id);
                    }
                }
            }
        }
#endif
    }
    LOG_W("unknow lcd!");
    return NULL;
}

void _lcd_low_level_init(void)
{
}

static rt_err_t api_lcd_init(rt_device_t dev)
{
    drv_lcd.brightness = MAX_BRIGHTNESS_LEVEL / 2;
    rt_sem_init(&drv_lcd.sem, "drv_lcd", 1, RT_IPC_FLAG_FIFO);


    drv_lcd.mq = rt_mq_create("drv_lcd", sizeof(LCD_MsgTypeDef), 4, RT_IPC_FLAG_FIFO);
    RT_ASSERT(drv_lcd.mq);

    rt_err_t ret = rt_thread_init(&drv_lcd.task, "lcd_task", lcd_task, drv_lcd.mq, drv_lcd_stack, sizeof(drv_lcd_stack),
                                  RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);

    RT_ASSERT(RT_EOK == ret);
    rt_thread_startup(&drv_lcd.task);
    rt_sem_init(&drv_lcd.draw_sem,     "lcd_draw", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&drv_lcd.sync_msg_sem, "lcd_msg", 0, RT_IPC_FLAG_FIFO);
#ifdef COPY2COMPRESS_FB_AND_SEND
    copy2compress_fb_init();
#endif /* COPY2COMPRESS_FB_AND_SEND */

    return RT_EOK;
}

rt_thread_t lcd_get_task(void)
{
    return &drv_lcd.task;
}

#ifdef LCD_USE_GPIO_TE
static void te_pin_irq_handler(void *arg)
{
    HAL_LCDC_TE_IRQHandler((LCDC_HandleTypeDef *) arg);
}
#endif /* LCD_USE_GPIO_TE */

#ifdef BSP_USE_LCDC2_ON_HPSYS
static void copy2buf(uint8_t *dst_buf, const uint8_t *src_buf, uint16_t format, int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
    uint32_t bytes_per_pixel;

    //Copy to LSPYS ram use memcpy

    switch (format)
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

    memcpy(dst_buf, src_buf,
           bytes_per_pixel * (x1 - x0 + 1) * (y1 - y0 + 1));

}
#endif /* BSP_USE_LCDC2_ON_HPSYS */

#ifdef BSP_USING_RAMLESS_LCD
static void *malloc_dma_friendly_sram(rt_size_t n)
{
    uint8_t *ret_p = rt_malloc(n);
    RT_ASSERT(ret_p != NULL);

    if (!IS_DMA_FRIENDLY_SRAM_RANGE(ret_p, n))
    {
#define list_max 1024
        uint8_t **malloc_list = (uint8_t **)rt_malloc(sizeof(uint8_t *) * list_max);
        RT_ASSERT(malloc_list != NULL);
        malloc_list[0] = ret_p;

        uint32_t malloc_cnt = 1;
        while (malloc_cnt < list_max)
        {
            ret_p = (uint8_t *)rt_malloc(n);
            RT_ASSERT(ret_p != NULL);
            malloc_list[malloc_cnt++] = ret_p;

            if (IS_DMA_FRIENDLY_SRAM_RANGE(ret_p, n)) break;
        }

        if (malloc_cnt >= list_max)
        {
            LOG_E("%s failed, size=%d", __FUNCTION__, n);
            RT_ASSERT(0);
        }

        //Free all malloced memory except 'ret_p'
        while (malloc_cnt > 0)
        {
            if (malloc_list[malloc_cnt - 1] != ret_p)
                rt_free(malloc_list[malloc_cnt - 1]);
            malloc_cnt--;
        }
        rt_free(malloc_list);

    }

    return (void *)ret_p;
}
static void allocate_ptc_code_buf(void)
{
    if (!ramless_code) ramless_code = (uint32_t *) malloc_dma_friendly_sram(sizeof(uint32_t) * RAMLESS_AUTO_REFR_CODE_SIZE_IN_WORD);
    RT_ASSERT(ramless_code != NULL);
#ifdef BSP_LCDC_USING_DPI
    if (!sram_data0) sram_data0 = (uint32_t *) malloc_dma_friendly_sram(sizeof(uint32_t) * SRAM_BUF_TOTAL_WORDS);
    RT_ASSERT(sram_data0 != NULL);
    if (!sram_data1) sram_data1 = (uint32_t *) malloc_dma_friendly_sram(sizeof(uint32_t) * SRAM_BUF_TOTAL_WORDS);
    RT_ASSERT(sram_data1 != NULL);
#endif /* BSP_LCDC_USING_DPI */
}
static void free_ptc_code_buf(void)
{
    if (ramless_code)
    {
        rt_free(ramless_code);
        ramless_code = NULL;
    }
#ifdef BSP_LCDC_USING_DPI
    if (sram_data0)
    {
        rt_free(sram_data0);
        sram_data0 = NULL;
    }
    if (sram_data1)
    {
        rt_free(sram_data1);
        sram_data1 = NULL;
    }
#endif /* BSP_LCDC_USING_DPI */
}

#endif /* BSP_USING_RAMLESS_LCD */

extern void list_mem(void);

static rt_err_t lcd_hw_open(void)
{
    LOG_I("HW open");
    BSP_LCD_PowerUp();


#ifdef QAD_SPI_USE_GPIO_CS
    gpio_cs_init();
#endif /* QAD_SPI_USE_GPIO_CS */

#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_start();
#endif  /* RT_USING_PM */

    if (NULL == drv_lcd.p_drv_ops)
    {
        drv_lcd.p_drv_ops = find_right_driver();
    }
    else
    {
        LOG_I("Init LCD %s", drv_lcd.p_drv_ops->name);

        if (drv_lcd.p_drv_ops->p_ops->Init != NULL)
            drv_lcd.p_drv_ops->p_ops->Init(&drv_lcd.hlcdc);
    }




    if (drv_lcd.p_drv_ops)
    {
        enable_low_power(&drv_lcd);

#ifdef DRV_LCD_USE_LCDC1
        HAL_NVIC_SetPriority(LCDC1_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(LCDC1_IRQn);
#else
        HAL_NVIC_SetPriority(LCDC2_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(LCDC2_IRQn);
#endif

#ifdef LCD_USE_GPIO_TE
        rt_pin_mode(LCD_USE_GPIO_TE, PIN_MODE_INPUT);
        rt_pin_attach_irq(LCD_USE_GPIO_TE, PIN_IRQ_MODE_RISING, te_pin_irq_handler, &drv_lcd.hlcdc);
        rt_pin_irq_enable(LCD_USE_GPIO_TE, 1);
#endif /* LCD_USE_GPIO_TE */
#ifdef BSP_LCDC1_USE_LCDC2_TE
        HAL_LCDC_UseLCDC2TE(&drv_lcd.hlcdc);
#endif /* BSP_LCDC1_USE_LCDC2_TE */

    }



#ifdef HAL_DSI_MODULE_ENABLED
    if (drv_lcd.p_drv_ops && HAL_LCDC_IS_DSI_IF(drv_lcd.hlcdc.Init.lcd_itf))
    {
        HAL_NVIC_SetPriority(DSIHOST_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(DSIHOST_IRQn);
    }
#endif /* HAL_DSI_MODULE_ENABLED */

#ifdef HAL_V2D_GPU_MODULE_ENABLED
    HAL_NVIC_SetPriority(V2D_GPU_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(V2D_GPU_IRQn);
#endif

#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_stop();
#endif  /* RT_USING_PM */

#ifdef BSP_USING_RAMLESS_LCD
    if ((drv_lcd.p_drv_ops) && (HAL_LCDC_IS_PTC_AUX_IF(drv_lcd.hlcdc.Init.lcd_itf)))
    {
        allocate_ptc_code_buf();
#ifdef BSP_LCDC_USING_DPI
        HAL_LCDC_RAMLESS_Init_Ext(&drv_lcd.hlcdc, ramless_code,
                                  (uint8_t *)&sram_data0[0], (uint8_t *)&sram_data1[0], SRAM_BUF_1LINE_WORDS * 4);

        /*
            'sram_data0' & 'sram_data1' are not retention memory,
             should setup it every time after HW power on.
        */
        sram_data0[SRAM_BUF_1LINE_WORDS] = SRAM_BUF_MAGIC_NUM;
        sram_data1[SRAM_BUF_1LINE_WORDS] = SRAM_BUF_MAGIC_NUM;

#else
        HAL_LCDC_RAMLESS_Init(&drv_lcd.hlcdc, ramless_code);
#endif /* BSP_LCDC_USING_DPI */
    }
#endif /* BSP_USING_RAMLESS_LCD */

    LOG_I("HW open done.");

    return RT_EOK;

}

static rt_err_t lcd_hw_close(void)
{
    LOG_I("HW close");


#ifdef LCD_USE_GPIO_TE
    rt_pin_irq_enable(LCD_USE_GPIO_TE, 0);
#endif /* LCD_USE_GPIO_TE */



    HAL_LCDC_DeInit(&drv_lcd.hlcdc);

    BSP_LCD_PowerDown();
#ifdef BSP_USING_RAMLESS_LCD
    free_ptc_code_buf();
#endif /* BSP_USING_RAMLESS_LCD */

    LOG_I("HW close done.");

    return RT_EOK;
}


static rt_err_t api_lcd_open(rt_device_t dev, rt_uint16_t oflag)
{
    LCD_MsgTypeDef msg;

    set_drv_lcd_state(LCD_STATUS_OPENING);

    msg.id = LCD_MSG_OPEN;
    msg.driver = &drv_lcd;

    send_msg_to_lcd_task(&msg);
    return RT_EOK;
}

static rt_err_t api_lcd_close(rt_device_t dev)
{
    LCD_MsgTypeDef msg;

    set_drv_lcd_state(LCD_STATUS_DISPLAY_OFF_PENDING);

    msg.id = LCD_MSG_CLOSE;
    msg.driver = &drv_lcd;

    send_msg_to_lcd_task(&msg);
    return RT_EOK;
}

static void print_u32_data(const char *name, uint32_t *p, uint32_t col_num, uint32_t total)
{
    uint32_t new_line = 0;
    rt_kprintf("\r\n%s(%08x):\r\n", name, p);

    while (total-- > 0)
    {
        rt_kprintf("%08x ", *p++);
        if (++new_line == col_num)
        {
            rt_kprintf("\r\n");
            new_line = 0;
        }
    }
}
static void lcd_driver_print_error_info(void)
{
    LOG_E("Error, Clear p_drv_ops %x, sem_v=%d", drv_lcd.p_drv_ops, drv_lcd.sem.value);

    /*Check possible error reasons*/
    if (drv_lcd.hlcdc.Instance)
    {

        rt_base_t mask;
        mask = rt_hw_interrupt_disable();
        rt_hw_interrupt_enable(mask);

        LOG_E("irq_en=%x", mask);
#ifdef DRV_LCD_USE_LCDC1
        LOG_E("LCDC1_irq_en=%x, active=%x, pending=%x", HAL_NVIC_GetEnableIRQ(LCDC1_IRQn), HAL_NVIC_GetActive(LCDC1_IRQn), NVIC_GetPendingIRQ(LCDC1_IRQn));
#else
        LOG_E("LCDC2_irq_en=%x, active=%x, pending=%x", HAL_NVIC_GetEnableIRQ(LCDC2_IRQn), HAL_NVIC_GetActive(LCDC2_IRQn), NVIC_GetPendingIRQ(LCDC2_IRQn));
#endif /* DRV_LCD_USE_LCDC1 */

        LOG_E("hlcdc errcode=%x, lock=%x, state=%x", drv_lcd.hlcdc.ErrorCode, drv_lcd.hlcdc.Lock, drv_lcd.hlcdc.State);
        LOG_E("LCDC STATUS=%x,TE=%x",    drv_lcd.hlcdc.Instance->STATUS, drv_lcd.hlcdc.Instance->TE_CONF);
        LOG_E("HAL dbg cnt %x, %x, %x", drv_lcd.hlcdc.debug_cnt0, drv_lcd.hlcdc.debug_cnt1, drv_lcd.hlcdc.debug_cnt2);
        LOG_E("DRV dbg cnt %x, %x, %x", drv_lcd.debug_cnt1, drv_lcd.debug_cnt2, drv_lcd.debug_cnt3);

#ifndef SOC_BF_Z0
        LOG_E("LCDC CANVAS TL=%x,BR=%x,SETTING=%x,IRQ=%x",
              drv_lcd.hlcdc.Instance->CANVAS_TL_POS,
              drv_lcd.hlcdc.Instance->CANVAS_BR_POS,
              drv_lcd.hlcdc.Instance->SETTING,
              drv_lcd.hlcdc.Instance->IRQ
             );

        LOG_E("LCDC LAYER0 CFG=%x,TL=%x,BR=%x,SRC=%x,DEC=%x",
              drv_lcd.hlcdc.Instance->LAYER0_CONFIG,
              drv_lcd.hlcdc.Instance->LAYER0_TL_POS,
              drv_lcd.hlcdc.Instance->LAYER0_BR_POS,
              drv_lcd.hlcdc.Instance->LAYER0_SRC,
              drv_lcd.hlcdc.Instance->LAYER0_DECOMP
             );

        LOG_E("LCDC LAYER1 CFG=%x,TL=%x,BR=%x,SRC=%x",
              drv_lcd.hlcdc.Instance->LAYER1_CONFIG,
              drv_lcd.hlcdc.Instance->LAYER1_TL_POS,
              drv_lcd.hlcdc.Instance->LAYER1_BR_POS,
              drv_lcd.hlcdc.Instance->LAYER1_SRC
             );

#endif
#ifdef HAL_DSI_MODULE_ENABLED
        if (HAL_LCDC_IS_DSI_IF(drv_lcd.hlcdc.Init.lcd_itf))
        {
            LOG_E("DSI write_finish=%d, D0_STOP=%d, is_master=%d, COM_INTF=%x, PHY_CTRL=%x",
                  __HAL_DSI_IS_WRITE_FINISH(&drv_lcd.hlcdc.hdsi),
                  __HAL_DSI_IS_DLANE0_STOP(&drv_lcd.hlcdc.hdsi),
                  __HAL_DSI_IS_MASTER_STATE(&drv_lcd.hlcdc.hdsi),
                  drv_lcd.hlcdc.hdsi.Instance->COM_INTF,
                  drv_lcd.hlcdc.hdsi.Instance->PHY_CTRL
                 );

            LOG_E("DSI INT_STAT1=%x,INT_STAT2=%x",
                  drv_lcd.hlcdc.hdsi.Instance->INT_STAT1,
                  drv_lcd.hlcdc.hdsi.Instance->INT_STAT2
                 );
        }
#endif /* HAL_DSI_MODULE_ENABLED */
#ifdef BSP_USING_RAMLESS_LCD
        if (HAL_LCDC_IS_PTC_AUX_IF(drv_lcd.hlcdc.Init.lcd_itf))
        {
            HAL_Delay(5);
            LOG_E("LCDC CANVAS TL=%x,BR=%x",
                  drv_lcd.hlcdc.Instance->CANVAS_TL_POS,
                  drv_lcd.hlcdc.Instance->CANVAS_BR_POS);

#ifdef SOC_BF0_HCPU
#ifdef SF32LB55X
            const uint32_t ptc_tab_size = 24;
            const uint32_t ptc_ch_words = 3;
            const uint32_t ptc_ch_cmp_len = 12;
#else
            const uint32_t ptc_tab_size = 32;
            const uint32_t ptc_ch_words = 4;
            const uint32_t ptc_ch_cmp_len = 12;
#endif /* SF32LB55X */

            LOG_E(" PTC_MEM4=%x, DMA_TC8=%x", hwp_ptc1->MEM4, hwp_dmac1->CM0AR8);


            LOG_E("BTIM1 CR=%x, PSC=%x,ARR=%x, EGR=%x", hwp_btim1->CR1, hwp_btim1->PSC, hwp_btim1->ARR, hwp_btim1->EGR);
            LOG_E("BTIM2 CR=%x, PSC=%x,ARR=%x, EGR=%x", hwp_btim2->CR1, hwp_btim2->PSC, hwp_btim2->ARR, hwp_btim2->EGR);

            if (drv_lcd.hlcdc.ptc_code[0] != drv_lcd.hlcdc.ptc_code[24])
            {
                LOG_E("ERR: ptc_code[0,24]=%08x, %08x", drv_lcd.hlcdc.ptc_code[0],
                      drv_lcd.hlcdc.ptc_code[24]);
            }

#ifdef hwp_busmon1
            if ((hwp_busmon1->MIN7 != (uint32_t)(&hwp_ptc1->MEM4)) || (hwp_busmon1->MAX7 != hwp_busmon1->MIN7 + 4))
            {
                LOG_E("ERR: BUS_MON7=[%x, %x]", hwp_busmon1->MIN7, hwp_busmon1->MAX7);
            }
#endif /* hwp_busmon1 */

            uint32_t dma_1_ch_regs = &hwp_dmac1->CCR2 - &hwp_dmac1->CCR1;
            print_u32_data("AllDMA", (uint32_t *) & (hwp_dmac1->CCR1), dma_1_ch_regs, dma_1_ch_regs * 8);

            {
                print_u32_data("regs", (uint32_t *) & (hwp_ptc1->TCR1), ptc_ch_words, ptc_tab_size);
                uint8_t phase_idx = 0;
                for (uint32_t *p = drv_lcd.hlcdc.ptc_code; p < drv_lcd.hlcdc.ptc_code +
                        RAMLESS_AUTO_REFR_CODE_SIZE_IN_WORD; p += ptc_tab_size)
                {
                    uint32_t *ptc_ch_code = p;
                    uint32_t *ptc_ch_reg = (uint32_t *) & (hwp_ptc1->TCR1);

                    for (uint32_t ch = 1; ch <= 8; ch++)
                    {
                        if (memcmp(ptc_ch_code, ptc_ch_reg, ptc_ch_cmp_len))
                        {
                            break;
                        }
                        else if (8 == ch)
                        {
                            LOG_E("code phase %d", phase_idx);
                            print_u32_data("code", p, ptc_ch_words, ptc_tab_size);
                        }
                        else
                        {
                            ptc_ch_reg  += ptc_ch_words;
                            ptc_ch_code += ptc_ch_words;
                        }
                    }

                    phase_idx++;
                }

            }

#endif /* SOC_BF0_HCPU */

        }
#endif /* BSP_USING_RAMLESS_LCD */
    }
}

static void lcd_driver_error_handle(void)
{
#ifdef DRV_LCD_USE_LCDC1
    HAL_NVIC_DisableIRQ(LCDC1_IRQn);
#else
    HAL_NVIC_DisableIRQ(LCDC2_IRQn);
#endif

#ifdef HAL_DSI_MODULE_ENABLED
    if (drv_lcd.p_drv_ops && HAL_LCDC_IS_DSI_IF(drv_lcd.hlcdc.Init.lcd_itf))
    {
        HAL_NVIC_DisableIRQ(DSIHOST_IRQn);
    }
#endif /* HAL_DSI_MODULE_ENABLED */

    drv_lcd.p_drv_ops = NULL;
    set_drv_lcd_state(LCD_STATUS_DISPLAY_TIMEOUT);
}

static void aysnc_send_cmplt_cbk(LCD_DrvTypeDef *p_drvlcd, const uint8_t *buffer)
{
    RT_ASSERT(p_drvlcd);

    rt_device_t p_lcd_device = &p_drvlcd->parent;

    p_drvlcd->debug_cnt3++;

    if (p_lcd_device->tx_complete)
    {
        p_lcd_device->tx_complete(p_lcd_device, (void *)buffer);
    }

}

typedef struct _lcd_mode_desc_s
{
    uint32_t buffer_color_format;
    uint32_t buffer_compression_rate;
    uint32_t lcd_dev_color_format;
} lcd_mode_desc_t;



/**
 * @brief
 * @param bright - [0~100]  0-close backlight， 100-all backlight on
 * @return
 */
static rt_err_t lcd_set_brightness(uint8_t bright)
{

    if (IS_DRV_LCD_ERROR()) return RT_ERROR;

    LOG_I("set brightness %d", bright);

    //In case of async_send_timeout_handler called before semaphore timeout
    if (drv_lcd.p_drv_ops && drv_lcd.p_drv_ops->p_ops->SetBrightness)
    {
        disable_low_power(&drv_lcd);
        drv_lcd.p_drv_ops->p_ops->SetBrightness(&drv_lcd.hlcdc, bright);
        enable_low_power(&drv_lcd);
    }
    drv_lcd.brightness = bright;


    return RT_EOK;
}


static rt_err_t api_lcd_control(rt_device_t dev, int cmd, void *args)
{
    if (LCD_STATUS_NONE == drv_lcd.status) //LCD not been opened.
    {
        return RT_EOK;
    }


    if (RTGRAPHIC_CTRL_GET_BUSY == cmd)
    {
        if (args)
        {
            rt_err_t result;

            /* try to take condition semaphore */
            result = rt_sem_trytake(&(drv_lcd.sem));
            if (result == -RT_ETIMEOUT)
            {
                /* it's timeout, busy state */
                *(bool *)args = true;
            }
            else if (result == RT_EOK)
            {
                /* has taken this semaphore, release it */
                rt_sem_release(&(drv_lcd.sem));

                /*Check async DMA copy*/
#if defined(COPY2COMPRESS_FB_AND_SEND) && !defined(BSP_USE_LCDC2_ON_HPSYS)
                result = rt_sem_trytake(&(compress_fb_ctx.dma_sema));
                if (result == -RT_ETIMEOUT)
                {
                    *(bool *)args = true;
                    return RT_EOK;
                }
                else if (result == RT_EOK)
                {
                    rt_sem_release(&(compress_fb_ctx.dma_sema));
                }
                else
                {
                    RT_ASSERT(0);
                }
#endif /* COPY2COMPRESS_FB_AND_SEND && !BSP_USE_LCDC2_ON_HPSYS*/

#ifdef BSP_USING_LCD_FRAMEBUFFER
                if (drv_lcd_fb_is_busy())
                {
                    *(bool *)args = true;
                    return RT_EOK;
                }
#endif /* BSP_USING_LCD_FRAMEBUFFER */



                *(bool *)args = (drv_lcd.mq->entry != 0) ? true : false;
            }
            else
            {
                RT_ASSERT(0);
            }


        }
        return RT_EOK;
    }

    if (SF_GRAPHIC_CTRL_GET_DRAW_ERR == cmd)
    {
        uint8_t *p_err = (uint8_t *)args;

        *p_err = drv_lcd.draw_error;

        if (*p_err)
        {
            LOG_I("Get error %d", *p_err);
            drv_lcd.draw_error = 0;
        }

        return RT_EOK;
    }

    LCD_MsgTypeDef msg;
    switch (cmd)
    {
    case RTGRAPHIC_CTRL_POWERON:
        msg.id = LCD_MSG_POWER_ON;
        break;

    case RTGRAPHIC_CTRL_POWEROFF:
        msg.id = LCD_MSG_POWER_OFF;
        break;

    case RTGRAPHIC_CTRL_SET_BRIGHTNESS:
        msg.id = LCD_MSG_SET_BRIGHTNESS;
        msg.content.brightness = *((uint8_t *)args);
        break;

    case RTGRAPHIC_CTRL_GET_BRIGHTNESS_ASYNC:
        msg.id = LCD_MSG_GET_BRIGHTNESS_ASYNC;
        msg.content.p_brightness_ret = (uint8_t *)args;
        break;

    case RTGRAPHIC_CTRL_SET_NEXT_TE:
        msg.id = LCD_MSG_SET_NEXT_TE;
        msg.content.TE_on = *((uint8_t *)args);
        break;

    case SF_GRAPHIC_CTRL_LCDC_FLUSH:
        drv_lcd.debug_cnt1++;
        msg.id = LCD_MSG_FLUSH_RECT_ASYNC;
        memcpy(&msg.content.flush, args, sizeof(msg.content.flush));
        break;

    case RTGRAPHIC_CTRL_SET_MODE:
        msg.id = LCD_MSG_SET_MODE;
        msg.content.idle_mode_on = *((uint8_t *)args);
        break;

    case SF_GRAPHIC_CTRL_ASSERT_IF_DRAWTIMEOUT:
        msg.id = LCD_MSG_CTRL_ASSERT_IF_DRAWTIMEOUT;
        msg.content.assert_timeout = (uint8_t)(uint32_t)args;
        break;

    case SF_GRAPHIC_CTRL_LCD_PRESENT:
        msg.id = LCD_MSG_CTRL_SET_LCD_PRESENT;
        msg.content.is_lcd_present = (uint8_t)(uint32_t)args;
        break;

    default:
        msg.id = LCD_MSG_INVALID;
        break;
    }

    if (msg.id != LCD_MSG_INVALID)
    {
        msg.driver = &drv_lcd;
        send_msg_to_lcd_task(&msg);
        return RT_EOK;
    }





    if (RT_EOK != api_lock(&drv_lcd))
    {
        WAIT_SEMA_TIMEOUT();
        return RT_ETIMEOUT;
    }

    switch (cmd)
    {
    case RTGRAPHIC_CTRL_GET_INFO:
    {
        struct rt_device_graphic_info *info;
        const LCDC_InitTypeDef *lcd_cfg;

        info = (struct rt_device_graphic_info *) args;
        RT_ASSERT(info != RT_NULL);
        memset(info, 0, sizeof(struct rt_device_graphic_info));

        if (NULL != drv_lcd.p_drv_ops)
        {
            lcd_cfg = drv_lcd.p_drv_ops->p_init_cfg;
            if (LCDC_PIXEL_FORMAT_RGB332 == lcd_cfg->color_mode)
            {
                info->bits_per_pixel = 8;
                info->pixel_format = RTGRAPHIC_PIXEL_FORMAT_RGB332;
            }
            else if (LCDC_PIXEL_FORMAT_RGB565 == lcd_cfg->color_mode)
            {
                info->bits_per_pixel = 16;
                info->pixel_format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
            }
            else if (LCDC_PIXEL_FORMAT_RGB666 == lcd_cfg->color_mode)
            {
                info->bits_per_pixel = 18;
                info->pixel_format = RTGRAPHIC_PIXEL_FORMAT_RGB666;
            }
            else if (LCDC_PIXEL_FORMAT_RGB888 == lcd_cfg->color_mode)
            {
                info->bits_per_pixel = 24;
                info->pixel_format = RTGRAPHIC_PIXEL_FORMAT_RGB888;
            }
            else
            {
                info->bits_per_pixel = 0;
                info->pixel_format = 0;
            }
            info->framebuffer = NULL;
            info->width = drv_lcd.p_drv_ops->lcd_horizonal_res;
            info->height = drv_lcd.p_drv_ops->lcd_vertical_res;
            info->draw_align = drv_lcd.p_drv_ops->pixel_align;
#ifdef BSP_USING_ROUND_TYPE_LCD
#ifdef BSP_USING_RECT_TYPE_LCD
#error "Can't define both 'ROUND_TYPE_LCD' and 'RECT_TYPE_LCD'"
#endif /* BSP_USING_RECT_TYPE_LCD */
            info->is_round   = 1;
#else
            info->is_round   = 0;
#endif /* BSP_USING_ROUND_TYPE_LCD */
            RT_ASSERT(info->draw_align > 0);

            switch (lcd_cfg->lcd_itf)
            {
            case LCDC_INTF_DBI_8BIT_A:
            case LCDC_INTF_DBI_8BIT_B:
            {
                info->bandwidth = lcd_cfg->freq << 3;
                break;
            }

            case LCDC_INTF_SPI_NODCX_1DATA:
            case LCDC_INTF_SPI_DCX_1DATA:
            {
                info->bandwidth = lcd_cfg->freq;
                break;
            }
            case LCDC_INTF_SPI_NODCX_2DATA:
            case LCDC_INTF_SPI_DCX_2DATA:
            {
                info->bandwidth = lcd_cfg->freq << 1;
                break;
            }
            case LCDC_INTF_SPI_NODCX_4DATA:
            case LCDC_INTF_SPI_DCX_4DATA:
            {
                info->bandwidth = lcd_cfg->freq << 2;
                break;
            }
            case LCDC_INTF_SPI_DCX_DDR_4DATA:
            {
                info->bandwidth = lcd_cfg->freq << 3;
                break;
            }
#ifdef HAL_DSI_MODULE_ENABLED
            case LCDC_INTF_DSI:
            case LCDC_INTF_DSI_VIDEO:
            {
                if (lcd_cfg->cfg.dsi.Init.NumberOfLanes == DSI_TWO_DATA_LANES)
                    info->bandwidth = lcd_cfg->freq << 1;
                else
                    info->bandwidth = lcd_cfg->freq;

                break;
            }
#endif /* HAL_DSI_MODULE_ENABLED */
            case LCDC_INTF_JDI_PARALLEL:
            {
                info->bandwidth = lcd_cfg->freq;
                break;
            }

            default:
            {
                info->bandwidth = 0;
                break;
            }
            }
        }
        else
        {
            info->bits_per_pixel = 16;          // To avoid assert.

            rt_uint8_t max_align_v = 1; //Get maximum align value according to LCD_HOR_RES_MAX
            rt_uint16_t v = LCD_HOR_RES_MAX;
            while (v)
            {
                if (v & 1) break;
                max_align_v = max_align_v << 1;
                v = v >> 1;
            }
            info->draw_align = max_align_v;
            info->width = LCD_HOR_RES_MAX;
            info->height = LCD_VER_RES_MAX;
        }

    }
    break;

    case RTGRAPHIC_CTRL_RECT_UPDATE:
        /* nothong to be done */
        break;



    case RT_DEVICE_CTRL_RESUME:
        if (NULL != drv_lcd.p_drv_ops)
        {
            HAL_StatusTypeDef err;
            //Re-initialize LCDC
            err = HAL_LCDC_Resume(&drv_lcd.hlcdc);
            RT_ASSERT(HAL_OK == err);
        }
#ifdef BSP_USING_EPIC
        drv_gpu_open();
#endif /* BSP_USING_EPIC */
        break;


    case RT_DEVICE_CTRL_SUSPEND:
#ifdef BSP_USING_EPIC
        drv_gpu_close();
#endif /* BSP_USING_EPIC */
        RT_ASSERT(HAL_LCDC_STATE_BUSY != drv_lcd.hlcdc.State);
        /*LCDC lost power supply*/
        break;



    case RTGRAPHIC_CTRL_GET_STATE:
    {
        RT_ASSERT(args != NULL);
        *((LCD_DrvStatusTypeDef *)args) = drv_lcd.status;
        break;
    }


    case RTGRAPHIC_CTRL_ROTATE_180:
    {
        if (LCD_ROTATE_0_DEGREE == drv_lcd.rotate)
            lcd_rotate(LCD_ROTATE_180_DEGREE);
        else
            lcd_rotate(LCD_ROTATE_0_DEGREE);
    }
    break;

    case RTGRAPHIC_CTRL_IRQ_INTERVAL_LINE:
    {
#ifdef LCDC_SUPPORT_LINE_DONE_IRQ
        drv_lcd.hlcdc.irq_lines = *((uint16_t *)args);
        LOG_D("Set irq interval line %d", drv_lcd.hlcdc.irq_lines);
#endif /* LCDC_SUPPORT_LINE_DONE_IRQ */
    }
    break;

    case RTGRAPHIC_CTRL_GET_BRIGHTNESS:
    {
        if (args)
        {
            *(uint8_t *)args = drv_lcd.brightness;
        }
    }
    break;


        /**********************************************/
        /******************Test only  START**********/
        /**********************************************/

#if 1

#ifdef COPY2COMPRESS_FB_AND_SEND
    case SF_GRAPHIC_CTRL_LCDC_CP2COMPRESS_FB_AND_SEND_CMPRATE_GET:
        if (args)
        {
            *((uint8_t *)args) = cmpr_rate;
        }
        break;

    case SF_GRAPHIC_CTRL_LCDC_CP2COMPRESS_FB_AND_SEND_CMPRATE_SET:
        if (args)
        {
            cmpr_rate = *((uint8_t *)args);
        }
        break;
#else
    case SF_GRAPHIC_CTRL_LCDC_CP2COMPRESS_FB_AND_SEND_CMPRATE_GET:
        if (args)
        {
            *((uint8_t *)args) = 0;
        }
        break;
#endif /* COPY2COMPRESS_FB_AND_SEND */


    case SF_GRAPHIC_CTRL_LCDC_OUT_FORMAT:
        if (args)
        {
            uint16_t color_f = *((uint16_t *)args);
            if (!IS_DRV_LCD_ERROR())
            {
                if (drv_lcd.p_drv_ops->p_ops->SetColorMode)
                {
                    disable_low_power(&drv_lcd);
                    drv_lcd.p_drv_ops->p_ops->SetColorMode(&drv_lcd.hlcdc, color_f);
                    enable_low_power(&drv_lcd);
                }
            }
        }
        break;
#endif

    /**********************************************/
    /******************Test only  END************/
    /**********************************************/

    default:
        rt_lcd_layer_control(&drv_lcd, cmd, args);
        break;
    }

    api_unlock(&drv_lcd);

    return RT_EOK;
}

static void api_lcd_set_pixel(const char *pixel, int x, int y)
{
    if (!IS_DRV_LCD_ERROR())
    {
        LCD_MsgTypeDef msg;

        msg.id = LCD_MSG_SET_PIXEL;
        msg.driver = &drv_lcd;
        msg.content.pixel.data = (const uint8_t *)pixel;
        msg.content.pixel.x = x;
        msg.content.pixel.y = y;

        send_msg_to_lcd_task(&msg);
    }
}

static void api_lcd_get_pixel(char *pixel, int x, int y)
{
    if (NULL == pixel)
    {
        return;
    }

    if (!IS_DRV_LCD_ERROR())
    {
        LCD_MsgTypeDef msg;

        msg.id = LCD_MSG_GET_PIXEL;
        msg.driver = &drv_lcd;
        msg.content.pixel.data = (const uint8_t *)pixel;
        msg.content.pixel.x = x;
        msg.content.pixel.y = y;

        send_msg_to_lcd_task(&msg);
    }
}


static rt_err_t draw_core(LCD_DrvTypeDef *p_drvlcd, const uint8_t *pixels, int x0, int y0, int x1, int y1)
{
    rt_err_t err = RT_ERROR;

    RT_ASSERT(pixels != NULL);
    RT_ASSERT((x0 <= x1) && (y0 <= y1));

    RT_ASSERT(0 == p_drvlcd->draw_lock);

    p_drvlcd->draw_lock = 1;
    if (IS_DRV_LCD_ERROR())
    {
        err = RT_EOK;//Do nothing if LCD ERROR.
    }
#ifdef SKIP_GPU_ERROR_FRAME
    else if (gpu_timeout_cnt)
    {
        gpu_timeout_cnt = 0;
        err = RT_EOK;//Do nothing if LCD ERROR.
    }
#endif /* SKIP_GPU_ERROR_FRAME */
    else if (p_drvlcd->p_drv_ops->p_ops->WriteMultiplePixels)
    {

#ifdef BSP_USE_LCDC2_ON_HPSYS

#ifndef LPSYS_FRAMEBUFFER_START_ADDR
        uint8_t *lcpu_buf = (uint8_t *)0x20440000;
        LOG_W("lcpu_buf 0x%x, TODO: define LPSYS buffer in mem_map.h", lcpu_buf);
#else
        uint8_t *lcpu_buf = (uint8_t *)LPSYS_FRAMEBUFFER_START_ADDR;
#endif
        /*LCDC2 can't access HPSYS memory, replace with lcpu buf here*/
        copy2buf(lcpu_buf, pixels, p_drvlcd->buf_format, x0, y0, x1, y1);
        pixels = (const uint8_t *)lcpu_buf;
#endif /* BSP_USE_LCDC2_ON_HPSYS */

        LOG_D("draw_core %x, [%d,%d,%d,%d]", pixels, x0, y0, x1, y1);


#ifdef RT_USING_PM
        rt_pm_request(PM_SLEEP_MODE_IDLE);
        rt_pm_hw_device_start();
#endif  /* RT_USING_PM */
        disable_low_power(p_drvlcd);



        int new_x0, new_x1, new_y0, new_y1;//Layer coordinates
        new_x0 = x0;
        new_x1 = x1;
        new_y0 = y0;
        new_y1 = y1;

        if (LCD_ROTATE_180_DEGREE == p_drvlcd->rotate)
        {
#ifdef LCDC_SUPPORT_H_MIRROR
            new_x0 = FLIP_V_BY_AREA(FLIP_V_BY_AREA(x0, 0, p_drvlcd->p_drv_ops->lcd_horizonal_res - 1), p_drvlcd->hlcdc.roi.x0, p_drvlcd->hlcdc.roi.x1);
            new_x1 = FLIP_V_BY_AREA(FLIP_V_BY_AREA(x1, 0, p_drvlcd->p_drv_ops->lcd_horizonal_res - 1), p_drvlcd->hlcdc.roi.x0, p_drvlcd->hlcdc.roi.x1);
#endif /* LCDC_SUPPORT_H_MIRROR */

#ifdef LCDC_SUPPORT_V_MIRROR
            new_y0 = FLIP_V_BY_AREA(y1, 0, p_drvlcd->p_drv_ops->lcd_vertical_res - 1);
            new_y1 = FLIP_V_BY_AREA(y0, 0, p_drvlcd->p_drv_ops->lcd_vertical_res - 1);
#endif /* LCDC_SUPPORT_V_MIRROR */

            RT_ASSERT((new_x0 <= new_x1) && (new_y0 <= new_y1));

            //Make ROI & Layer's coordinate positive
            int off_x, off_y;
            off_x = (new_x0 < 0) ? (0 - new_x0) : 0;
            off_y = (new_y0 < 0) ? (0 - new_y0) : 0;

            if ((off_x > 0) || (off_y > 0))
            {
                p_drvlcd->hlcdc.roi.x0 += off_x;
                p_drvlcd->hlcdc.roi.y0 += off_y;
                p_drvlcd->hlcdc.roi.x1 += off_x;
                p_drvlcd->hlcdc.roi.y1 += off_y;

                new_x0 += off_x;
                new_y0 += off_y;
                new_x1 += off_x;
                new_y1 += off_y;
            }
        }

        err = RT_EOK;
        if (p_drvlcd->p_drv_ops->p_ops->ESDDetect != NULL)
        {
            uint32_t cur_tick = rt_tick_get();
            //Check ESD every 3 seconds
            if (HAL_GetElapsedTick(p_drvlcd->last_esd_check_tick, cur_tick) > rt_tick_from_millisecond(3000))
            {
                uint32_t ret_v = p_drvlcd->p_drv_ops->p_ops->ESDDetect(&p_drvlcd->hlcdc);

                if (ret_v != 0)
                {
                    LOG_E("ESD error %d", ret_v);
                    err = -RT_ETIMEOUT;//Go to timeout handler
                }

                p_drvlcd->last_esd_check_tick = cur_tick;
            }
        }

#ifdef BSP_USING_RAMLESS_LCD
#ifdef BSP_LCDC_USING_DPI
        if (HAL_LCDC_IS_PTC_AUX_IF(p_drvlcd->hlcdc.Init.lcd_itf))
        {
            if ((sram_data0[SRAM_BUF_1LINE_WORDS] != SRAM_BUF_MAGIC_NUM)
                    || (sram_data1[SRAM_BUF_1LINE_WORDS] != SRAM_BUF_MAGIC_NUM))
            {
                LOG_E("RAMLESS DPI overwritten!!");
                RT_ASSERT(0);
            }
        }
#endif
#endif


        //Start async send
        if (RT_EOK == err)
        {
            p_drvlcd->hlcdc.XferCpltCallback = SendLayerDataCpltCbk;
            p_drvlcd->hlcdc.XferErrorCallback = SendLayerDataErrCbk;
            p_drvlcd->hlcdc.debug_cnt0++;
#ifdef LCDC_SUPPORT_LINE_DONE_IRQ
            p_drvlcd->hlcdc.XferLineCallback = SendLineDoneCpltCbk;
#endif /* LCDC_SUPPORT_LINE_DONE_IRQ */

#ifdef QAD_SPI_USE_GPIO_CS
            gpio_cs_enable();
#endif /* QAD_SPI_USE_GPIO_CS */

            //rt_enter_critical();//In case of task switch between 'get tick' and 'WriteMultiplePixels'
            p_drvlcd->start_tick = rt_tick_get();
            p_drvlcd->p_drv_ops->p_ops->WriteMultiplePixels(&p_drvlcd->hlcdc, pixels, new_x0, new_y0, new_x1, new_y1);
            //rt_exit_critical();


            /* --------- Wait send compelete -----------------*/
            err = rt_sem_take(&p_drvlcd->draw_sem, MAX_LCD_DRAW_TIME);



#ifdef QAD_SPI_USE_GPIO_CS
            gpio_cs_disable();
#endif /* QAD_SPI_USE_GPIO_CS */

            //Clear all callback
            p_drvlcd->hlcdc.XferCpltCallback = NULL;
            p_drvlcd->hlcdc.XferErrorCallback = NULL;
#ifdef LCDC_SUPPORT_LINE_DONE_IRQ
            p_drvlcd->hlcdc.XferLineCallback = NULL;
#endif /* LCDC_SUPPORT_LINE_DONE_IRQ */

        }



        if (RT_EOK == err)
        {
            p_drvlcd->timeout_retry_cnt = MAX_TIMEOUT_RETRY;
            p_drvlcd->end_tick = rt_tick_get();
            if (p_drvlcd->send_time_log)
            {
                LOG_I("draw_core buf=%x done, cost=%d(rt_ticks)", pixels, (p_drvlcd->end_tick - p_drvlcd->start_tick));
            }

            /*
                Turn on display if it not, and the last line of LCD should be updated.
            */
            if ((LCD_STATUS_INITIALIZED == p_drvlcd->status) && (y1 >= p_drvlcd->p_drv_ops->lcd_vertical_res - 1))
            {
                LOG_I("Auto turn on display.");
                lcd_set_brightness(p_drvlcd->brightness);
                lcd_display_on();
            }
        }
        else if (-RT_ETIMEOUT == err)
        {
            LOG_E("draw_core timeout");

            /*Reset 'draw_sem' in case of 'XferCpltCallback' invoked after timeout*/
            rt_err_t err2 = rt_sem_control(&p_drvlcd->draw_sem, RT_IPC_CMD_RESET, 0);
            RT_ASSERT(RT_EOK == err2);

            p_drvlcd->draw_error = 1;
            lcd_driver_print_error_info();
            rt_thread_mdelay(30);//Wait LCDC finish if it was just started.
            lcd_driver_print_error_info();

            if (p_drvlcd->p_drv_ops->p_ops->TimeoutDbg != NULL)
                p_drvlcd->p_drv_ops->p_ops->TimeoutDbg(&p_drvlcd->hlcdc);

            if (1 == p_drvlcd->assert_timeout)
            {
                //Stop and assert
                set_drv_lcd_state(LCD_STATUS_DISPLAY_TIMEOUT);

                lcd_driver_error_handle();
                LOG_E("lcd_timeout_error %s, %d", __FUNCTION__, __LINE__);
                RT_ASSERT(0);
            }
            else if ((2 == p_drvlcd->assert_timeout) || (0 == p_drvlcd->timeout_retry_cnt))
            {
                LOG_E("Clear p_drv_ops");
                p_drvlcd->p_drv_ops = NULL;
                p_drvlcd->timeout_retry_cnt = MAX_TIMEOUT_RETRY;
                set_drv_lcd_state(LCD_STATUS_NOT_FIND_LCD);

                err = RT_EOK; //Not return error here
            }
            else
            {
                //Reset LCD and LCDC
                RT_ASSERT(p_drvlcd->p_drv_ops != NULL);
                RT_ASSERT(p_drvlcd->p_drv_ops->p_ops->TimeoutReset != NULL);

                LOG_E("draw_core timeout reset LCD START(%d)", p_drvlcd->timeout_retry_cnt);


                p_drvlcd->p_drv_ops->p_ops->TimeoutReset(&p_drvlcd->hlcdc);

                set_drv_lcd_state(LCD_STATUS_INITIALIZED);

                //SendLayerDataCpltCbk(&drv_lcd.hlcdc);
                LOG_E("draw_core timeout reset LCD  END");
                if (p_drvlcd->timeout_retry_cnt > 0) p_drvlcd->timeout_retry_cnt--;
                err = RT_EOK; //Not return error here
            }
        }
        else
        {
            RT_ASSERT(0); //Wait sema err;
        }

        enable_low_power(p_drvlcd);
#ifdef RT_USING_PM
        rt_pm_release(PM_SLEEP_MODE_IDLE);
        rt_pm_hw_device_stop();
#endif  /* RT_USING_PM */

    }


    p_drvlcd->draw_lock = 0;

    return err;
}


static void api_lcd_draw_rect(const char *pixels, int x0, int y0, int x1, int y1)
{

    if (IS_DRV_LCD_ERROR()) return;
    RT_ASSERT(!IS_LCDC_OFF());

#if 1
    LCD_MsgTypeDef msg;

    msg.id = LCD_MSG_DRAW_RECT;
    msg.driver = &drv_lcd;
    msg.content.draw_ctx.pixels = (const uint8_t *)pixels;
    msg.content.draw_ctx.area.x0 = x0;
    msg.content.draw_ctx.area.y0 = y0;
    msg.content.draw_ctx.area.x1 = x1;
    msg.content.draw_ctx.area.y1 = y1;

    send_msg_to_lcd_task(&msg);


    return;

#elif 0
    rt_err_t err;
    err = api_lock(&drv_lcd);
    if (RT_EOK != err)
    {
        WAIT_SEMA_TIMEOUT();
        return;
    }

    LOG_D("api_lcd_draw_rect %x, [%d,%d,%d,%d]", pixels, x0, y0, x1, y1);
    err = draw_core(&drv_lcd, pixels, x0, y0, x1, y1);
    RT_ASSERT(RT_EOK == err);

    api_unlock(&drv_lcd);

    return;


#else
    if (drv_lcd.p_drv_ops->p_ops->WriteMultiplePixels)
    {
        rt_err_t err;
        LOG_D("api_lcd_draw_rect %x, [%d,%d,%d,%d]", pixels, x0, y0, x1, y1);

        err = api_lock(&drv_lcd);
        if (RT_EOK != err)
        {
            WAIT_SEMA_TIMEOUT();
            return;
        }

#ifdef QAD_SPI_USE_GPIO_CS
        gpio_cs_enable();
#endif /* QAD_SPI_USE_GPIO_CS */

#ifdef RT_USING_PM
        rt_pm_request(PM_SLEEP_MODE_IDLE);
        rt_pm_hw_device_start();
#endif  /* RT_USING_PM */

        RT_ASSERT((x0 <= x1) && (y0 <= y1));

        drv_lcd.hlcdc.XferCpltCallback = SendLayerDataCpltCbk;
        drv_lcd.hlcdc.XferErrorCallback = SendLayerDataErrCbk;
        drv_lcd.hlcdc.debug_cnt0++;
        drv_lcd.p_drv_ops->p_ops->WriteMultiplePixels(&drv_lcd.hlcdc, (const uint8_t *)pixels, x0, y0, x1, y1);

        err = api_lock(&drv_lcd);
        if (RT_EOK != err)
        {
#ifdef RT_USING_PM
            rt_pm_release(PM_SLEEP_MODE_IDLE);
            rt_pm_hw_device_stop();

#endif  /* RT_USING_PM */

#ifdef QAD_SPI_USE_GPIO_CS
            gpio_cs_disable();
#endif /* QAD_SPI_USE_GPIO_CS */

            WAIT_SEMA_TIMEOUT();
            return;
        }

        api_unlock(&drv_lcd);

#if 0 //Will been relased in SendLayerDataCpltCbk
#ifdef RT_USING_PM
        rt_pm_release(PM_SLEEP_MODE_IDLE);
        rt_pm_hw_device_stop();

#endif  /* RT_USING_PM */

#ifdef QAD_SPI_USE_GPIO_CS
        gpio_cs_disable();
#endif /* QAD_SPI_USE_GPIO_CS */
#endif /* 0 */

        /*
            Turn on display if it not, and the last line of LCD should be updated.
        */
        if ((LCD_STATUS_INITIALIZED == drv_lcd.status) && (y1 >= drv_lcd.p_drv_ops->lcd_vertical_res - 1))
        {
            LOG_I("Auto turn on display after draw rect");
            lcd_set_brightness(drv_lcd.brightness);
            lcd_display_on();
        }
    }

#endif
}

static void api_lcd_draw_rect_async(const char *pixels, int x0, int y0, int x1, int y1)
{

    drv_lcd.debug_cnt1++;

    if (IS_DRV_LCD_ERROR())
    {
        aysnc_send_cmplt_cbk(&drv_lcd, (uint8_t *)pixels);
        return;
    }

#if 1
    LCD_MsgTypeDef msg;

    msg.id = LCD_MSG_DRAW_RECT_ASYNC;
    msg.driver = &drv_lcd;
    msg.content.draw_ctx.pixels = (const uint8_t *)pixels;
    msg.content.draw_ctx.area.x0 = x0;
    msg.content.draw_ctx.area.y0 = y0;
    msg.content.draw_ctx.area.x1 = x1;
    msg.content.draw_ctx.area.y1 = y1;

    send_msg_to_lcd_task(&msg);

    return;


#elif 0
    RT_ASSERT(!IS_LCDC_OFF());
    rt_err_t err;
    err = api_lock(&drv_lcd);
    if (RT_EOK != err)
    {
        WAIT_SEMA_TIMEOUT();
        aysnc_send_cmplt_cbk(&drv_lcd, (uint8_t *)pixels);
        return;
    }

    LOG_D("api_lcd_draw_rect_async %x, [%d,%d,%d,%d]", pixels, x0, y0, x1, y1);
    err = draw_core(pixels, x0, y0, x1, y1);
    RT_ASSERT(RT_EOK == err);

    return;

#else
    if (drv_lcd.p_drv_ops->p_ops->WriteMultiplePixels)
    {
        rt_err_t err;

        LOG_D("api_lcd_draw_rect_async %x, [%d,%d,%d,%d]", pixels, x0, y0, x1, y1);

#ifdef RT_USING_PM
        rt_pm_request(PM_SLEEP_MODE_IDLE);
        rt_pm_hw_device_start();
#endif  /* RT_USING_PM */

        err = api_lock(&drv_lcd);
        if (RT_EOK != err)
        {
            WAIT_SEMA_TIMEOUT();
            goto ASYNC_SEND_FAIL;
        }

#ifdef QAD_SPI_USE_GPIO_CS
        gpio_cs_enable();
#endif /* QAD_SPI_USE_GPIO_CS */

        RT_ASSERT((x0 <= x1) && (y0 <= y1));

        rt_enter_critical();
        rt_timer_start(&drv_lcd.async_time);

        drv_lcd.hlcdc.XferCpltCallback = SendLayerDataCpltCbk;
        drv_lcd.hlcdc.XferErrorCallback = SendLayerDataErrCbk;
        drv_lcd.hlcdc.debug_cnt0++;
        drv_lcd.p_drv_ops->p_ops->WriteMultiplePixels(&drv_lcd.hlcdc, (const uint8_t *)pixels, x0, y0, x1, y1);
        rt_exit_critical();

        /*
            Turn on display if it not, and the last line of LCD should be updated.
        */
        if ((LCD_STATUS_INITIALIZED == drv_lcd.status) && (y1 >= drv_lcd.p_drv_ops->lcd_vertical_res - 1))
        {
            //Wait WriteMultiplePixels done.
            {
                err = api_lock(&drv_lcd);
                if (RT_EOK != err)
                {
                    WAIT_SEMA_TIMEOUT();
                    goto ASYNC_SEND_FAIL;
                }
                api_unlock(&drv_lcd);
            }

            LOG_I("Auto turn on display after draw rect async");
            lcd_set_brightness(drv_lcd.brightness);
            lcd_display_on();
        }

        return;
    }



ASYNC_SEND_FAIL:
    {
        SendLayerDataCpltCbk(&drv_lcd.hlcdc);
    }
#endif

}

static void set_window(int x0, int y0, int x1, int y1)
{
    if (!IS_DRV_LCD_ERROR())
    {
        if (drv_lcd.p_drv_ops->p_ops->SetRegion)
        {
            int new_x0, new_x1, new_y0, new_y1;


            disable_low_power(&drv_lcd);

            if (drv_lcd.send_time_log)
            {
                LOG_I("gap %02d ms, set_window x0x1y0y1[%d,%d,%d,%d]",
                      (rt_tick_get() - drv_lcd.end_tick) * 1000 / RT_TICK_PER_SECOND, //elapsed ms util last draw_core done.
                      x0, x1, y0, y1);
            }
            new_x0 = x0;
            new_x1 = x1;
            new_y0 = y0;
            new_y1 = y1;

            if (LCD_ROTATE_180_DEGREE == drv_lcd.rotate)
            {
#ifdef LCDC_SUPPORT_H_MIRROR
                new_x0 = FLIP_V_BY_AREA(x1, 0, drv_lcd.p_drv_ops->lcd_horizonal_res - 1);
                new_x1 = FLIP_V_BY_AREA(x0, 0, drv_lcd.p_drv_ops->lcd_horizonal_res - 1);
#endif /* LCDC_SUPPORT_H_MIRROR */

#ifdef LCDC_SUPPORT_V_MIRROR
                new_y0 = FLIP_V_BY_AREA(y1, 0, drv_lcd.p_drv_ops->lcd_vertical_res - 1);
                new_y1 = FLIP_V_BY_AREA(y0, 0, drv_lcd.p_drv_ops->lcd_vertical_res - 1);
#endif /* LCDC_SUPPORT_V_MIRROR */
            }

            RT_ASSERT((new_x0 <= new_x1) && (new_y0 <= new_y1));
            RT_ASSERT((new_x1 - new_x0 + 1) <= drv_lcd.p_drv_ops->lcd_horizonal_res);
            RT_ASSERT((new_y1 - new_y0 + 1) <= drv_lcd.p_drv_ops->lcd_vertical_res);

            drv_lcd.p_drv_ops->p_ops->SetRegion(&drv_lcd.hlcdc, new_x0, new_y0, new_x1, new_y1);
            enable_low_power(&drv_lcd);

        }
    }
}



static void api_lcd_set_window(int x0, int y0, int x1, int y1)
{
    if (!IS_DRV_LCD_ERROR())
    {
#if 1
        LCD_MsgTypeDef msg;

        msg.id = LCD_MSG_SET_WINDOW;
        msg.driver = &drv_lcd;
        msg.content.window.x0 = x0;
        msg.content.window.y0 = y0;
        msg.content.window.x1 = x1;
        msg.content.window.y1 = y1;

        send_msg_to_lcd_task(&msg);

#else
        rt_err_t err;
        err = api_lock(&drv_lcd);
        if (RT_EOK == err)
        {
            set_window(x0, y0, x1, y1);
        }
        else
        {
            WAIT_SEMA_TIMEOUT();
        }
        api_unlock(&drv_lcd);
#endif /* 0 */
    }
}


static void lcd_display_on(void)
{
    if (!IS_DRV_LCD_ERROR())
    {
        LOG_I("display on");

        disable_low_power(&drv_lcd);
        if (drv_lcd.p_drv_ops->p_ops->DisplayOn)
        {
            drv_lcd.p_drv_ops->p_ops->DisplayOn(&drv_lcd.hlcdc);
        }
        enable_low_power(&drv_lcd);

        set_drv_lcd_state(LCD_STATUS_DISPLAY_ON);
    }
}

static void lcd_display_off(void)
{
    if (!IS_DRV_LCD_ERROR())
    {
        LOG_I("display off");

        disable_low_power(&drv_lcd);
        if (drv_lcd.p_drv_ops->p_ops->SetBrightness)
        {
            drv_lcd.p_drv_ops->p_ops->SetBrightness(&drv_lcd.hlcdc, MIN_BRIGHTNESS_LEVEL);
        }
        if (drv_lcd.p_drv_ops->p_ops->DisplayOff)
        {
            drv_lcd.p_drv_ops->p_ops->DisplayOff(&drv_lcd.hlcdc);
        }
        enable_low_power(&drv_lcd);
        HAL_LCDC_DeInit(&drv_lcd.hlcdc);

        set_drv_lcd_state(LCD_STATUS_DISPLAY_OFF);
    }
}

static void lcd_set_idle_status(uint8_t status)
{
    lcd_idle_status = status;
}

L1_RET_CODE_SECT(lcd_get_idle_status, uint8_t lcd_get_idle_status(void))
{
    return lcd_idle_status;
}


static void lcd_idlemode_on(void)
{
    if (!IS_DRV_LCD_ERROR())
    {
        if (drv_lcd.p_drv_ops->p_ops->IdleModeOn)
        {
            disable_low_power(&drv_lcd);
            drv_lcd.p_drv_ops->p_ops->IdleModeOn(&drv_lcd.hlcdc);
            enable_low_power(&drv_lcd);
            lcd_set_idle_status(1);
        }

        set_drv_lcd_state(LCD_STATUS_IDLE_MODE);
    }
}


static void lcd_idlemode_off(void)
{
#if 1
    if (!IS_DRV_LCD_ERROR())
    {
        if (drv_lcd.p_drv_ops->p_ops->IdleModeOff)
        {
            disable_low_power(&drv_lcd);
            drv_lcd.p_drv_ops->p_ops->IdleModeOff(&drv_lcd.hlcdc);
            enable_low_power(&drv_lcd);
            lcd_set_idle_status(0);
        }

        set_drv_lcd_state(LCD_STATUS_DISPLAY_ON);
    }
#else
    BSP_LCD_PowerDown();

    /*
        Restart LCD if LCD power control pin or reset pin belong to GPIOA,
        because GPIOA can not keeping output voltage when wokeup from standby mode on A0.
    */
#ifdef HAL_EPIC_MODULE_ENABLED
    epic = NULL;
#endif


    rt_timer_detach(&drv_lcd.async_time);

    rt_sem_detach(&drv_lcd.sem);

#ifdef COPY2COMPRESS_FB_AND_SEND
    copy2compress_fb_deinit();
#endif /* COPY2COMPRESS_FB_AND_SEND */
    api_lcd_open(NULL, 0);
    /* Display On */
    lcd_display_on();

#endif /* 0 */
}


#if  0//def RT_USING_PM
static const uint8_t lcd_brightness[PM_SLEEP_MODE_MAX] =
{
    100,
    100,
    100,
    50,
    0,
    0,
};

static int lcd_suspend(const struct rt_device *device, uint8_t mode)
{
    int r = RT_EOK;

    if (drv_lcd.brightness != lcd_brightness[mode])
    {
        lcd_set_brightness(lcd_brightness[mode]);
        drv_lcd.brightness = lcd_brightness[mode];
    }

    return r;
}

void lcd_resume(const struct rt_device *device, uint8_t mode)
{
    if (drv_lcd.brightness != lcd_brightness[PM_SLEEP_MODE_IDLE])
    {
        lcd_set_brightness(lcd_brightness[PM_SLEEP_MODE_IDLE]);
        drv_lcd.brightness = lcd_brightness[PM_SLEEP_MODE_IDLE];
    }
    return ;
}


static const struct rt_device_pm_ops lcd_pm_op =
{
    .suspend = lcd_suspend,
    .resume = lcd_resume,
};
#endif


static void lcd_rotate(uint16_t degree)
{
    if (!IS_DRV_LCD_ERROR())
    {
        LOG_I("Rotate %d degree", degree);
        disable_low_power(&drv_lcd);
        if (drv_lcd.p_drv_ops->p_ops->Rotate)
            drv_lcd.p_drv_ops->p_ops->Rotate(&drv_lcd.hlcdc, (LCD_DrvRotateTypeDef)degree);
        HAL_LCDC_LayerRotate(&drv_lcd.hlcdc, drv_lcd.select_layer, (HAL_LCDC_RotateDef)degree);
        drv_lcd.rotate = (LCD_DrvRotateTypeDef)degree;
        enable_low_power(&drv_lcd);
    }
}


static void lcd_task(void *param)
{
    rt_mq_t msg_queue = (rt_mq_t) param;
    LCD_MsgTypeDef msg;
    rt_err_t err;

    while (1)
    {
        LCD_DrvTypeDef *p_drvlcd;
        rt_tick_t start_tick;

        err = rt_mq_recv(msg_queue, &msg, sizeof(msg), RT_WAITING_FOREVER);

        RT_ASSERT(RT_EOK == err);
        p_drvlcd = msg.driver;

        start_tick = rt_tick_get();

        LOG_D("lcd_task exec msg%x: [%s].", msg.tick, lcd_msg_to_name(msg.id));
#ifdef RT_USING_PM
        rt_pm_request(PM_SLEEP_MODE_IDLE);
        rt_pm_hw_device_start();
#endif  /* RT_USING_PM */

        err = api_lock(p_drvlcd);
        if (RT_EOK == err)
        {
            switch (msg.id)
            {
            case LCD_MSG_OPEN:
            {
                LOG_I("open");

#ifdef BSP_USING_EPIC
                drv_gpu_open();
#endif /* BSP_USING_EPIC */
                lcd_hw_open();


                /*Set LCDC background*/
                HAL_LCDC_SetBgColor(&drv_lcd.hlcdc, 0, 0, 0);

                /*Set default layer configuration*/
                HAL_LCDC_LayerReset(&drv_lcd.hlcdc, HAL_LCDC_LAYER_DEFAULT);

                if (drv_lcd.p_drv_ops)
                {
                    set_drv_lcd_state(LCD_STATUS_INITIALIZED);
                }
                else
                {
                    LOG_I("Close LCD to save power");
                    lcd_hw_close();
                    set_drv_lcd_state(LCD_STATUS_NOT_FIND_LCD);
                }


                LOG_I("open done.");
            }
            break;

            case LCD_MSG_CLOSE:
            {
                LOG_I("close");
#ifdef BSP_USING_EPIC
                drv_gpu_close();
#endif /* BSP_USING_EPIC */
                lcd_display_off();
                lcd_hw_close();
                LOG_I("close done.");
            }
            break;

            case LCD_MSG_POWER_ON:
            {
                LOG_I("Power on");

#ifdef BSP_USING_EPIC
                drv_gpu_open();//Keep gpu_open before lcd_hw_open. Issue-2314
#endif /* BSP_USING_EPIC */
                lcd_hw_open();
                if (p_drvlcd->p_drv_ops)
                {
                    set_drv_lcd_state(LCD_STATUS_INITIALIZED); //Auto set backlight after send first framebuffer
                }
                else
                {
                    LOG_I("Close LCD to save power");
                    lcd_hw_close();
                    set_drv_lcd_state(LCD_STATUS_NOT_FIND_LCD);
                }
                LOG_I("Power on done.");

            }
            break;


            case LCD_MSG_POWER_OFF:
            {
                LOG_I("Power off");
#ifdef BSP_USING_EPIC
                drv_gpu_close();
#endif /* BSP_USING_EPIC */
                lcd_display_off();
                lcd_hw_close();
                LOG_I("Power off done");
            }
            break;

            case LCD_MSG_SET_MODE:
            {
                LOG_I("idle mode on=%d", msg.content.idle_mode_on);

                if (msg.content.idle_mode_on)
                {
                    lcd_idlemode_on();
                }
                else
                {
                    lcd_idlemode_off();
                }
            }
            break;



            case LCD_MSG_CTRL_SET_LCD_PRESENT:
            {
                LOG_I("Set lcd present %d", msg.content.is_lcd_present);

                drv_lcd.force_lcd_missing = !msg.content.is_lcd_present;
                drv_lcd.p_drv_ops = NULL;
                set_drv_lcd_state(LCD_STATUS_NOT_FIND_LCD);
            }
            break;

            case LCD_MSG_CTRL_ASSERT_IF_DRAWTIMEOUT:
            {
                LOG_I("Set assert_timeout %d", msg.content.assert_timeout);

                drv_lcd.assert_timeout = msg.content.assert_timeout;
            }
            break;

            case LCD_MSG_DRAW_RECT_ASYNC:
            {
                RT_ASSERT(!IS_LCDC_OFF());
                LCD_DrawCtxDef *p_drawctx = &msg.content.draw_ctx;

                LOG_D("api_lcd_draw_rect_async %x, [%d,%d,%d,%d]", p_drawctx->pixels,
                      p_drawctx->area.x0, p_drawctx->area.y0,
                      p_drawctx->area.x1, p_drawctx->area.y1);
                err = draw_core(p_drvlcd, p_drawctx->pixels,
                                p_drawctx->area.x0, p_drawctx->area.y0,
                                p_drawctx->area.x1, p_drawctx->area.y1);
                RT_ASSERT(RT_EOK == err);

                aysnc_send_cmplt_cbk(p_drvlcd, p_drawctx->pixels);
            }
            break;

            case LCD_MSG_DRAW_RECT:
            {
                RT_ASSERT(!IS_LCDC_OFF());
                LCD_DrawCtxDef *p_drawctx = &msg.content.draw_ctx;

                LOG_D("api_lcd_draw_rect %x, [%d,%d,%d,%d]", p_drawctx->pixels,
                      p_drawctx->area.x0, p_drawctx->area.y0,
                      p_drawctx->area.x1, p_drawctx->area.y1);
                err = draw_core(p_drvlcd, p_drawctx->pixels,
                                p_drawctx->area.x0, p_drawctx->area.y0,
                                p_drawctx->area.x1, p_drawctx->area.y1);
                RT_ASSERT(RT_EOK == err);

            }
            break;

#ifdef COPY2COMPRESS_FB_AND_SEND
            case LCD_MSG_DRAW_COMP_RECT_ASYNC:
            {
                RT_ASSERT(!IS_LCDC_OFF());
                frame_buf_t *compressed_buffer = (frame_buf_t *)msg.content.compress_buf;


                err = rt_sem_take(&(compressed_buffer->sema), rt_tick_from_millisecond(MAX_LCD_DRAW_TIME));

                if (RT_EOK == err)
                {
                    LOG_D("comressed_buf set_window [%d,%d,%d,%d]",
                          compressed_buffer->window.x0, compressed_buffer->window.y0,
                          compressed_buffer->window.x1, compressed_buffer->window.y1);
                    set_window(compressed_buffer->window.x0, compressed_buffer->window.y0,
                               compressed_buffer->window.x1, compressed_buffer->window.y1);

                    HAL_LCDC_LayerSetCmpr(&p_drvlcd->hlcdc, p_drvlcd->select_layer, cmpr_rate);

                    EventStartA(3);

                    LOG_D("comressed_buf draw %x, [%d,%d,%d,%d]", compressed_buffer->addr,
                          compressed_buffer->fb_rect.x0, compressed_buffer->fb_rect.y0,
                          compressed_buffer->fb_rect.x1, compressed_buffer->fb_rect.y1);
                    err = draw_core(p_drvlcd, (const uint8_t *)compressed_buffer->addr,
                                    compressed_buffer->fb_rect.x0, compressed_buffer->fb_rect.y0,
                                    compressed_buffer->fb_rect.x1, compressed_buffer->fb_rect.y1);
                    RT_ASSERT(RT_EOK == err);

                    /*Close compress layer after compress buf send done.
                        In case send uncompressed buf through 'draw_rect' API after
                    */
                    HAL_LCDC_LayerSetCmpr(&p_drvlcd->hlcdc, p_drvlcd->select_layer, 0);

                    //Free compressed buf
                    {
                        //RT_ASSERT(compress_fb_ctx.flush_busy);
                        EventStopA(3);
                        LOG_D("comressed_buf free %x", compressed_buffer->addr);
                        free_frame_buf(compressed_buffer);
                    }
                    err = rt_sem_release(&(compressed_buffer->sema));
                    RT_ASSERT(RT_EOK == err);
                }
                else
                {
                    LOG_E("%d, skiped compressed_buf %x, elapsed %d ticks", err, compressed_buffer->addr,
                          rt_tick_get() - compressed_buffer->start_tick);

                    free_frame_buf(compressed_buffer);
                }

            }
            break;
#endif /* COPY2COMPRESS_FB_AND_SEND */

            case LCD_MSG_FLUSH_RECT_ASYNC:
            {
                RT_ASSERT(!IS_LCDC_OFF());
                lcd_flush_info_t *flush_info = (lcd_flush_info_t *)&msg.content.flush;

                if (flush_info->cmpr_rate > 0)
                {
                    int32_t new_x1 = flush_info->window.x1;
                    if (new_x1 < flush_info->pixel_area.x1)
                    {
                        //The end of compressed fb should be decoded.
                        new_x1 = flush_info->pixel_area.x1;
                        new_x1 = RT_ALIGN(new_x1, p_drvlcd->p_drv_ops->pixel_align) - 1;

                        if ((new_x1 > flush_info->pixel_area.x1)
                                && (new_x1 < p_drvlcd->p_drv_ops->lcd_horizonal_res))
                        {
                            LOG_E("Background%d,%d,%d",
                                  new_x1, flush_info->window.x1, flush_info->pixel_area.x1);
                        }

                        flush_info->window.x1 = new_x1;
                    }
                }


                LOG_D("set_window [%d,%d,%d,%d]",
                      flush_info->window.x0, flush_info->window.y0,
                      flush_info->window.x1, flush_info->window.y1);
                set_window(flush_info->window.x0, flush_info->window.y0,
                           flush_info->window.x1, flush_info->window.y1);

                HAL_LCDC_LayerSetCmpr(&p_drvlcd->hlcdc, p_drvlcd->select_layer, flush_info->cmpr_rate);


                LOG_D("draw %x, [%d,%d,%d,%d]", flush_info->pixel,
                      flush_info->pixel_area.x0, flush_info->pixel_area.y0,
                      flush_info->pixel_area.x1, flush_info->pixel_area.y1);
                err = draw_core(p_drvlcd, (const uint8_t *)flush_info->pixel,
                                flush_info->pixel_area.x0, flush_info->pixel_area.y0,
                                flush_info->pixel_area.x1, flush_info->pixel_area.y1);
                RT_ASSERT(RT_EOK == err);

                /*Close compress layer after compress buf send done.
                    In case send uncompressed buf through 'draw_rect' API after
                */
                HAL_LCDC_LayerSetCmpr(&p_drvlcd->hlcdc, p_drvlcd->select_layer, 0);


                aysnc_send_cmplt_cbk(p_drvlcd, flush_info->pixel);
            }
            break;


            case LCD_MSG_SET_WINDOW:
            {
                RT_ASSERT(!IS_LCDC_OFF());
                LCD_AreaDef    *p_window = &msg.content.window;

                LOG_D("lcd_set_window [%d,%d,%d,%d]",
                      p_window->x0, p_window->y0, p_window->x1, p_window->y1);
                set_window(p_window->x0, p_window->y0, p_window->x1, p_window->y1);

            }
            break;

            case LCD_MSG_SET_BRIGHTNESS:
            {
                lcd_set_brightness(msg.content.brightness);
                lcd_display_on();
            }
            break;

            case LCD_MSG_GET_BRIGHTNESS_ASYNC:
            {
                if (msg.content.p_brightness_ret)
                {
                    LOG_D("GET_BRIGHTNESS_ASYNC %p, %d", msg.content.p_brightness_ret, p_drvlcd->brightness);

                    *msg.content.p_brightness_ret = p_drvlcd->brightness;
                }
            }
            break;

            case LCD_MSG_SET_PIXEL:
            {
                RT_ASSERT(!IS_LCDC_OFF());
                if (p_drvlcd->p_drv_ops->p_ops->WritePixel)
                {
                    disable_low_power(p_drvlcd);
                    p_drvlcd->p_drv_ops->p_ops->WritePixel(&p_drvlcd->hlcdc,
                                                           msg.content.pixel.x,
                                                           msg.content.pixel.y,
                                                           msg.content.pixel.data);
                    enable_low_power(p_drvlcd);
                }
            }
            break;


            case LCD_MSG_SET_NEXT_TE:
            {
                uint8_t TE_on = msg.content.TE_on;
                LOG_D("TE_on=%d", TE_on);
                HAL_LCDC_Next_Frame_TE(&drv_lcd.hlcdc, (0 == TE_on) ? false : true);
            }
            break;


            case LCD_MSG_GET_PIXEL:
            {
                RT_ASSERT(!IS_LCDC_OFF());
                if (p_drvlcd->p_drv_ops->p_ops->ReadPixel)
                {
                    disable_low_power(p_drvlcd);
                    uint32_t v = p_drvlcd->p_drv_ops->p_ops->ReadPixel(&p_drvlcd->hlcdc,
                                 msg.content.pixel.x,
                                 msg.content.pixel.y);
                    enable_low_power(p_drvlcd);

                    *(uint32_t *)msg.content.pixel.data = v;
                }

            }
            break;


            default:
                RT_ASSERT(0);
                break;
            }


            api_unlock(p_drvlcd);
        }
        else
        {
            WAIT_SEMA_TIMEOUT();
            if (LCD_MSG_DRAW_RECT_ASYNC == (msg.id))
            {
                aysnc_send_cmplt_cbk(p_drvlcd, (uint8_t *)msg.content.draw_ctx.pixels);
            }
        }

#ifdef RT_USING_PM
        rt_pm_release(PM_SLEEP_MODE_IDLE);
        rt_pm_hw_device_stop();
#endif  /* RT_USING_PM */

        if (IS_SYNC_MSG_ID(msg.id))
        {
            err = rt_sem_release(&p_drvlcd->sync_msg_sem);
            RT_ASSERT(RT_EOK == err);
        }

        LOG_D("lcd_task exec msg%x: [%s] cost=%dms.", msg.tick, lcd_msg_to_name(msg.id), rt_tick_get() - start_tick);
    }

}


static int rt_hw_lcd_init(void)
{
    static const struct rt_device_graphic_ops ops =
    {
        api_lcd_set_pixel,
        api_lcd_get_pixel,
        NULL,
        NULL,
        NULL,
#ifdef COPY2COMPRESS_FB_AND_SEND
        api_copy2compress_fb_and_send_sync,
        api_copy2compress_fb_and_send_async,
        api_compress_fb_set_window,
#else
        api_lcd_draw_rect,
        api_lcd_draw_rect_async,
        api_lcd_set_window,
#endif /* COPY2COMPRESS_FB_AND_SEND */
    };

    _lcd_low_level_init();

    memset(&drv_lcd, 0, sizeof(drv_lcd));

#ifdef DRV_LCD_USE_LCDC1
    drv_lcd.hlcdc.Instance = LCDC1;
#else
    drv_lcd.hlcdc.Instance = LCDC2;
#endif

    drv_lcd.select_layer = HAL_LCDC_LAYER_DEFAULT;

    /* register lcd device */
    {
        rt_device_t p_lcd_device;
        p_lcd_device = &drv_lcd.parent;
        p_lcd_device->type = RT_Device_Class_Graphic;
        p_lcd_device->init = api_lcd_init;
        p_lcd_device->open = api_lcd_open;
        p_lcd_device->close = api_lcd_close;
        p_lcd_device->control = api_lcd_control;
        p_lcd_device->read = RT_NULL;
        p_lcd_device->write = RT_NULL;

        p_lcd_device->user_data = (void *)&ops;

        /* register graphic device driver */
        rt_device_register(p_lcd_device, "lcd",
                           RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
#if 0//def RT_USING_PM
        rt_pm_device_register(p_lcd_device, &lcd_pm_op);
#endif
    }


    drv_lcd.status = LCD_STATUS_NONE;
    drv_lcd.auto_lowpower = 1;
    drv_lcd.draw_lock = 0;
    drv_lcd.rotate = LCD_ROTATE_0_DEGREE;
    drv_lcd.assert_timeout = 1;
    drv_lcd.timeout_retry_cnt = MAX_TIMEOUT_RETRY;



    return 0;
}
INIT_BOARD_EXPORT(rt_hw_lcd_init);


#ifdef COPY2COMPRESS_FB_AND_SEND

static void copy2compress_fb_init(void)
{
    compress_fb_ctx.frame_buf[0].addr = compress_buf1;
    compress_fb_ctx.frame_buf[0].writetimes = 0;
    rt_sem_init(&compress_fb_ctx.frame_buf[0].sema, "cfbsem0", 1, RT_IPC_FLAG_FIFO);

#if (COMPRESSED_FRAME_BUF_NUM > 1)
    compress_fb_ctx.frame_buf[1].addr = compress_buf2;
    compress_fb_ctx.frame_buf[1].writetimes = 0;
    rt_sem_init(&compress_fb_ctx.frame_buf[1].sema, "cfbsem1", 1, RT_IPC_FLAG_FIFO);
#endif

    rt_sem_init(&compress_fb_ctx.dma_sema, "cfbdma", 1, RT_IPC_FLAG_FIFO);


}

static void copy2compress_fb_deinit(void)
{

    rt_sem_detach(&compress_fb_ctx.dma_sema);
    rt_sem_detach(&compress_fb_ctx.frame_buf[0].sema);
#if (COMPRESSED_FRAME_BUF_NUM > 1)
    rt_sem_detach(&compress_fb_ctx.frame_buf[1].sema);
#endif
}



static void copy2compress_fb_dma_done_cb(void)
{
    rt_err_t err;

    frame_buf_t *compressed_buffer = compress_fb_ctx.dma_copy_buf;
    compressed_buffer->cp_done_tick = rt_tick_get();

#ifndef BSP_USE_LCDC2_ON_HPSYS
    err = rt_sem_release(&compress_fb_ctx.dma_sema);
    RT_ASSERT(RT_EOK == err);

    EventStopA(2);
#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_stop();

#endif  /* RT_USING_PM */
#endif /* BSP_USE_LCDC2_ON_HPSYS */


    //4. Check compressed buffer
    RT_ASSERT(COMPRESS_BUFF_MAGIC_FLAG == compressed_buffer->addr[COMPRESS_BUF_SIZE_IN_WORDS]); //Check overflow


    err = rt_sem_release(&(compressed_buffer->sema));
    RT_ASSERT(RT_EOK == err);



    if (1 == compressed_buffer->writetimes) //Send msg only first time, ignore overwrite msg. And prevent lcd_task mq overflow
    {
        LCD_MsgTypeDef msg;

        msg.id = LCD_MSG_DRAW_COMP_RECT_ASYNC;
        msg.driver = &drv_lcd;
        msg.content.compress_buf = compressed_buffer;
        send_msg_to_lcd_task(&msg);

        LOG_D("send msg%x LCD_MSG_DRAW_COMP_RECT_ASYNC %x.", msg.tick, compressed_buffer->addr);
    }


    //Pixels were copy to comressed buf, release it.
    aysnc_send_cmplt_cbk(&drv_lcd, (uint8_t *)compress_fb_ctx.dma_copy_src_buf);

}

static void dma_err_cb(void)
{
    LOG_E("dma_err_cb extdma error(%x).", EXT_DMA_GetError());

    //RT_ASSERT(0);
}



/* Allocate available frame buffer for write  */
static frame_buf_t *alloc_frame_buf(void)
{
    frame_buf_t *buf = RT_NULL;
    uint32_t i;

    rt_enter_critical(); //In case of sem changed if switch to lcd_task
    for (i = 0; i < COMPRESSED_FRAME_BUF_NUM; i++)
    {
        buf = &compress_fb_ctx.frame_buf[i];

        if (RT_EOK == rt_sem_trytake(&buf->sema))
        {
            break;
        }
    }
    rt_exit_critical();

    if (COMPRESSED_FRAME_BUF_NUM == i)
    {
#if (1 == COMPRESSED_FRAME_BUF_NUM)
        LOG_D("Wait the only 1 cmpbuf free");
        rt_err_t err = rt_sem_take(&(buf->sema), rt_tick_from_millisecond(MAX_LCD_DRAW_TIME));
        if (RT_EOK != err)
        {
            WAIT_SEMA_TIMEOUT();
            return NULL;
        }
#else
        RT_ASSERT(0); //Always get a buf which is not been flushing
#endif
    }

    return buf;
}

/* release frame buffer */
static void free_frame_buf(frame_buf_t *buf)
{


    buf->writetimes = 0;
    buf->end_tick = rt_tick_get();



}

#if 0
/* get frame buffer for flushing */
static frame_buf_t *get_pending_frame_buf(void)
{
    frame_buf_t *buf = RT_NULL;

    if (FRAME_BUF_EMPTY != get_frame_buf_status())
    {
        if (compress_fb_ctx.frame_buf[compress_fb_ctx.rd_idx].data_ready)
        {
            buf = &compress_fb_ctx.frame_buf[compress_fb_ctx.rd_idx];
        }
    }

    return buf;
}
#endif /* 0 */

static frame_buf_t *copy2compress_fb_async(const char *pixels, int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
    rt_err_t err;



    EXT_DMA_CmprTypeDef cmpr;
    uint32_t bytes_per_pixel;
    frame_buf_t *free_buffer;


    RT_ASSERT((x0 < x1) && (y0 < y1));

#ifndef BSP_USE_LCDC2_ON_HPSYS
#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_start();
#endif  /* RT_USING_PM */

    err = rt_sem_take(&compress_fb_ctx.dma_sema, rt_tick_from_millisecond(1000));
    RT_ASSERT(RT_EOK == err);
#endif /* BSP_USE_LCDC2_ON_HPSYS */


    //1.Allocate compressed buffer
    EventStartA(1);
    free_buffer = alloc_frame_buf();
    RT_ASSERT(NULL != free_buffer);

    LOG_D("Allocated compress_fb %x", free_buffer->addr);
    EventStopA(1);


    free_buffer->start_tick = rt_tick_get();
    free_buffer->writetimes++;


    //2. Initial compressed buffer
    if (free_buffer->writetimes > 1)
    {
        //Join old window with new one
        free_buffer->window.x0 = MIN(free_buffer->window.x0, compress_fb_ctx.tmp_window.x0);
        free_buffer->window.y0 = MIN(free_buffer->window.y0, compress_fb_ctx.tmp_window.y0);
        free_buffer->window.x1 = MAX(free_buffer->window.x1, compress_fb_ctx.tmp_window.x1);
        free_buffer->window.y1 = MAX(free_buffer->window.y1, compress_fb_ctx.tmp_window.y1);
    }
    else
    {
        memcpy(&(free_buffer->window), &(compress_fb_ctx.tmp_window), sizeof(free_buffer->window));
    }

    free_buffer->fb_rect.x0 = x0;
    free_buffer->fb_rect.y0 = y0;
    free_buffer->fb_rect.x1 = x1;
    free_buffer->fb_rect.y1 = y1;
    free_buffer->cmpr_rate = cmpr.cmpr_rate;
    free_buffer->addr[COMPRESS_BUF_SIZE_IN_WORDS] = COMPRESS_BUFF_MAGIC_FLAG;




    //3. Copy the pixels to compressed buffer
    LOG_D("copy2compress_fb_async window [%d,%d,%d,%d]",
          free_buffer->window.x0, free_buffer->window.y0,
          free_buffer->window.x1, free_buffer->window.y1);
    LOG_D("copy2compress_fb_async %x->%x, buf[%d,%d,%d,%d]", pixels, free_buffer->addr, x0, y0, x1, y1);
#ifdef BSP_USE_LCDC2_ON_HPSYS
    //Copy to LSPYS ram use memcpy
    copy2buf(free_buffer->addr, pixels, drv_lcd.buf_format, x0, y0, x1, y1);
    compress_fb_ctx.dma_copy_src_buf = pixels;
    compress_fb_ctx.dma_copy_buf = free_buffer;

    copy2compress_fb_dma_done_cb();
#else

    cmpr.cmpr_rate = cmpr_rate;
    if (cmpr.cmpr_rate > 0)
    {
        cmpr.cmpr_en = true;
    }
    else
    {
        cmpr.cmpr_en = false;
    }

    switch (drv_lcd.buf_format)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        cmpr.src_format = EXTDMA_CMPRCR_SRCFMT_RGB565;
        bytes_per_pixel  = 2;
        break;
    case RTGRAPHIC_PIXEL_FORMAT_RGB888:
        cmpr.src_format = EXTDMA_CMPRCR_SRCFMT_RGB888;
        bytes_per_pixel = 3;
        break;

    default:
        RT_ASSERT(0);
        break;
    }

    RT_ASSERT(cmpr.src_format == COMPRESSED_FRAME_BUF_FMT);

    cmpr.col_num = x1 - x0 + 1;
    cmpr.row_num = y1 - y0 + 1;
    RT_ASSERT(MAX_LINEBUFFER_BYTES >= cmpr.col_num * bytes_per_pixel);
    RT_ASSERT(MAX_LINE_NUM >= cmpr.row_num);

    err = EXT_DMA_ConfigCmpr(1, 1, &cmpr);
    RT_ASSERT(RT_EOK == err);

    EXT_DMA_Register_Callback(EXT_DMA_XFER_CPLT_CB_ID, copy2compress_fb_dma_done_cb);
    EXT_DMA_Register_Callback(EXT_DMA_XFER_ERROR_CB_ID, dma_err_cb);
    EventStartA(2);
    compress_fb_ctx.dma_copy_src_buf = pixels;
    compress_fb_ctx.dma_copy_buf = free_buffer;
    err = EXT_DMA_START_ASYNC((uint32_t)pixels, (uint32_t)free_buffer->addr, RT_ALIGN(bytes_per_pixel * cmpr.col_num * cmpr.row_num, 4) / 4);
    RT_ASSERT(RT_EOK == err);

#endif /* BSP_USE_LCDC2_ON_HPSYS */


    return free_buffer;
}


static void api_copy2compress_fb_and_send_sync(const char *pixels, int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
    /**/
    api_lcd_set_window(compress_fb_ctx.tmp_window.x0,
                       compress_fb_ctx.tmp_window.y0,
                       compress_fb_ctx.tmp_window.x1,
                       compress_fb_ctx.tmp_window.y1);


    api_lcd_draw_rect(pixels, x0, y0, x1, y1);
}


static void api_copy2compress_fb_and_send_async(const char *pixels, int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
    drv_lcd.debug_cnt1++;

    if (!IS_DRV_LCD_ERROR())
    {
        //Invoke 'copy2compress_fb_xxx' in current thread
        copy2compress_fb_async(pixels, x0, y0, x1, y1);
    }
    else
    {
        //None LCD or LCD error, do nothing, call cmplt cbk directly.
        aysnc_send_cmplt_cbk(&drv_lcd, (uint8_t *)pixels);
    }

}


static void api_compress_fb_set_window(int x0, int y0, int x1, int y1)
{
    if (IS_DRV_LCD_ERROR()) return;

    RT_ASSERT((x0 <= x1) && (y0 <= y1));
    RT_ASSERT((x1 - x0 + 1) <= drv_lcd.p_drv_ops->lcd_horizonal_res);
    RT_ASSERT((y1 - y0 + 1) <= drv_lcd.p_drv_ops->lcd_vertical_res);

    compress_fb_ctx.tmp_window.x0 = x0;
    compress_fb_ctx.tmp_window.y0 = y0;
    compress_fb_ctx.tmp_window.x1 = x1;
    compress_fb_ctx.tmp_window.y1 = y1;
}


#endif /* COPY2COMPRESS_FB_AND_SEND */


LCDC_HandleTypeDef *debug_get_drv_lcd_handler(void)
{
    return &drv_lcd.hlcdc;
}


#ifdef LCD_USING_PWM_AS_BACKLIGHT
static struct rt_device lcd_backlight_device;
static uint8_t lcd_backlight_level;       // save local bl, check previous level

static rt_size_t backligt_get(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    if (buffer != NULL)
    {
        *((uint8_t *) buffer)  =  lcd_backlight_level;

        struct rt_device_pwm *backlight_device = (struct rt_device_pwm *)rt_device_find(LCD_PWM_BACKLIGHT_INTERFACE_NAME);
        if (!backlight_device)
        {
            RT_ASSERT(0);
        }
        else
        {


            struct rt_pwm_configuration configuration = {0};
            configuration.channel = LCD_PWM_BACKLIGHT_CHANEL_NUM;
            if (RT_EOK == rt_device_control(&backlight_device->parent, PWM_CMD_GET, &configuration))
            {
                LOG_I("pwm_config ch=%d,peroid=%d,pulse=%d",
                      configuration.channel,
                      configuration.period,
                      configuration.pulse);
            }
        }

        LOG_I("backligt_get %d%", lcd_backlight_level);

        return 1;
    }

    return 0;
}

static rt_size_t backligt_set(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    if (buffer != NULL)
    {
        uint8_t percent = *((uint8_t *) buffer);

        if (percent > 100) percent = 100;


        struct rt_device_pwm *backlight_device = (struct rt_device_pwm *)rt_device_find(LCD_PWM_BACKLIGHT_INTERFACE_NAME);
        if (!backlight_device)
        {
            RT_ASSERT(0);
        }
        else
        {
            rt_uint32_t period = 1 * 1000 * 1000;
            LOG_I("backligt_set %d%", percent);

            rt_pwm_set(backlight_device, LCD_PWM_BACKLIGHT_CHANEL_NUM, period, period * percent / 100);
            if (0 == percent)
                rt_pwm_disable(backlight_device, LCD_PWM_BACKLIGHT_CHANEL_NUM);
            else
                rt_pwm_enable(backlight_device, LCD_PWM_BACKLIGHT_CHANEL_NUM);
        }

        lcd_backlight_level = percent;

        return 1;
    }

    return 0;

}

static int rt_hw_lcd_backlight_init(void)
{
    memset(&lcd_backlight_device, 0, sizeof(lcd_backlight_device));

    lcd_backlight_device.type = RT_Device_Class_Char;
    lcd_backlight_device.read = backligt_get;
    lcd_backlight_device.write = backligt_set;


    /* register graphic device driver */
    return rt_device_register(&lcd_backlight_device, "lcdlight",
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);

}

INIT_BOARD_EXPORT(rt_hw_lcd_backlight_init);
#ifdef RT_USING_FINSH
static rt_err_t get_lcd_pwm_bglit(int argc, char **argv)
{
    uint8_t level = 0;
    backligt_get(&lcd_backlight_device, 0, &level, 1);

    LOG_I("level=%d", level);
    return 0;
}

MSH_CMD_EXPORT(get_lcd_pwm_bglit, get_lcd_pwm_bglit);
#endif

#endif /* LCD_USING_PWM_AS_BACKLIGHT */


#ifdef RT_USING_FINSH

#define DEBUG_PRINTF(...)   LOG_E(__VA_ARGS__)

/*
static rt_err_t api_lcd_init(int argc, char **argv)
{
    if (IS_DRV_LCD_ERROR()) return  RT_EOK;

    drv_lcd.p_drv_ops->p_ops->Init(&drv_lcd.hlcdc);

    return 0;
}

MSH_CMD_EXPORT(api_lcd_init, api_lcd_init);
*/

static rt_err_t lcd_rreg(int argc, char **argv)
{
    uint32_t reg, reg_len, data_len;
    uint32_t read_freq = 2 * 1000 * 1000;
    uint32_t data = 0;

    HAL_StatusTypeDef hal_err;
    rt_err_t rt_err;

    if (argc < 4)
    {
        DEBUG_PRINTF("lcd_rreg <reg_addr> <reg_addr_len> <data_len>");
        return 0;
    }

    reg       = strtoul(argv[1], 0, 16);
    reg_len   = strtoul(argv[2], 0, 10);
    data_len  = strtoul(argv[3], 0, 10);
    if (argc > 4)
    {
        read_freq = strtoul(argv[4], 0, 10);
    }

    if ((0 == reg_len) || (reg_len > 4))
    {
        DEBUG_PRINTF("reg_len should be 1~4");
        return 0;
    }
    else if ((reg >> (reg_len * 8)) != 0)
    {
        DEBUG_PRINTF("reg_addr[0x%x] overflow? Maximum reg_addr_len is %d byte(s).", reg, reg_len);
        return 0;
    }


    DEBUG_PRINTF("lcd_rreg%d [%x] %d(byte)", reg_len, reg, data_len);
    rt_err = api_lock(&drv_lcd);
    RT_ASSERT(RT_EOK == rt_err);
    disable_low_power(&drv_lcd);

#ifdef BSP_USING_RAMLESS_LCD
    rt_base_t level = rt_hw_interrupt_disable(); //In case RAMLESS FSM is running

    uint8_t ramless_running = drv_lcd.hlcdc.running;
    if (ramless_running)
    {
        HAL_LCDC_RAMLESS_Stop(&drv_lcd.hlcdc);
    }
#endif /* BSP_USING_RAMLESS_LCD */


    if (HAL_LCDC_IS_SPI_IF(drv_lcd.hlcdc.Init.lcd_itf) || HAL_LCDC_IS_DBI_IF(drv_lcd.hlcdc.Init.lcd_itf))
    {
        DEBUG_PRINTF("SPI/DBI r_freq=%d", read_freq);
        uint32_t prev_freq = drv_lcd.hlcdc.Init.freq;

        HAL_LCDC_SetFreq(&drv_lcd.hlcdc, read_freq); //read mode min cycle 300ns
        hal_err = HAL_LCDC_ReadDatas(&drv_lcd.hlcdc, reg, reg_len, (uint8_t *)&data, data_len);
        HAL_LCDC_SetFreq(&drv_lcd.hlcdc, prev_freq); //Restore normal frequency
    }
    else
    {
        hal_err = HAL_LCDC_ReadDatas(&drv_lcd.hlcdc, reg, reg_len, (uint8_t *)&data, data_len);
    }

#ifdef BSP_USING_RAMLESS_LCD
    if (ramless_running)
    {
        HAL_LCDC_RAMLESS_Start(&drv_lcd.hlcdc);
    }
    rt_hw_interrupt_enable(level);
#endif /* BSP_USING_RAMLESS_LCD */
    enable_low_power(&drv_lcd);

    api_unlock(&drv_lcd);

    if (HAL_OK == hal_err)
        DEBUG_PRINTF("result=%08x\n", data);
    else
        DEBUG_PRINTF("ERROR=%d\n", hal_err);

    return 0;
}
MSH_CMD_EXPORT(lcd_rreg, lcd_rreg);


static rt_err_t lcd_wreg(int argc, char **argv)
{
    uint32_t reg, reg_len, data_len;
    uint8_t   parameter[4];
    HAL_StatusTypeDef hal_err;
    rt_err_t rt_err;

    if (argc < 3)
    {
        DEBUG_PRINTF("lcd_wreg <reg_addr> <reg_addr_len> [data0] [data1] ... [data3]");
        return 0;
    }

    reg       = strtoul(argv[1], 0, 16);
    reg_len   = strtoul(argv[2], 0, 10);
    data_len  = argc - 3;
    if ((0 == reg_len) || (reg_len > 4))
    {
        DEBUG_PRINTF("reg_len should be 1~4");
        return 0;
    }
    else if ((reg >> (reg_len * 8)) != 0)
    {
        DEBUG_PRINTF("reg_addr[0x%x] overflow? Maximum reg_addr_len is %d byte(s).", reg, reg_len);
        return 0;
    }


    for (uint32_t i = 3; i < argc; i++)
    {
        parameter[i - 3] = strtoul(argv[i], 0, 16);
    }

    DEBUG_PRINTF("lcd_wreg%d reg[%x] %d(byte)", reg_len, reg, data_len);
    rt_err = api_lock(&drv_lcd);
    RT_ASSERT(RT_EOK == rt_err);

    disable_low_power(&drv_lcd);
#ifdef BSP_USING_RAMLESS_LCD
    rt_base_t level = rt_hw_interrupt_disable(); //In case RAMLESS FSM is running

    uint8_t ramless_running = drv_lcd.hlcdc.running;
    if (ramless_running)
    {
        HAL_LCDC_RAMLESS_Stop(&drv_lcd.hlcdc);
    }
#endif /* BSP_USING_RAMLESS_LCD */

    hal_err = HAL_LCDC_WriteDatas(&drv_lcd.hlcdc, reg, reg_len, parameter, data_len);

#ifdef BSP_USING_RAMLESS_LCD
    if (ramless_running)
    {
        HAL_LCDC_RAMLESS_Start(&drv_lcd.hlcdc);
    }
    rt_hw_interrupt_enable(level);
#endif /* BSP_USING_RAMLESS_LCD */
    enable_low_power(&drv_lcd);
    api_unlock(&drv_lcd);


    if (HAL_OK == hal_err)
        DEBUG_PRINTF("Done.\n");
    else
        DEBUG_PRINTF("ERROR=%d\n", hal_err);

    return 0;

}
MSH_CMD_EXPORT(lcd_wreg, lcd_wreg);


static rt_err_t lcd_ctrl(int argc, char **argv)
{
    if (argc < 2)
    {
        DEBUG_PRINTF("lcd_ctrl [OPTION] [VALUE]\n");
        DEBUG_PRINTF("    OPTION:on,off,br\n");
        return RT_EOK;
    }

    if (!drv_lcd.p_drv_ops)
    {
        DEBUG_PRINTF("lcd device not found\n");
    }


    if (strcmp(argv[1], "open") == 0)
    {
        api_lcd_init(&drv_lcd.parent);
        api_lcd_open(&drv_lcd.parent, 0);
    }
    else if (strcmp(argv[1], "close") == 0)
    {
        api_lcd_close(&drv_lcd.parent);
    }
    else if (strcmp(argv[1], "on") == 0)
    {
        api_lcd_control(&drv_lcd.parent, RTGRAPHIC_CTRL_POWERON, NULL);
    }
    else if (strcmp(argv[1], "off") == 0)
    {
        api_lcd_control(&drv_lcd.parent, RTGRAPHIC_CTRL_POWEROFF, NULL);
    }
    else if (strcmp(argv[1], "off_on") == 0)
    {
        api_lcd_control(&drv_lcd.parent, RTGRAPHIC_CTRL_POWEROFF, NULL);
        api_lcd_control(&drv_lcd.parent, RTGRAPHIC_CTRL_POWERON, NULL);
    }
    else if (strcmp(argv[1], "br") == 0)
    {
        if (argc > 2)
        {
            uint32_t v = strtoul(argv[2], 0, 10);
            api_lcd_control(&drv_lcd.parent, RTGRAPHIC_CTRL_SET_BRIGHTNESS, &v);
        }
        else
        {
            rt_device_t device = rt_device_find("lcdlight");
            if (device)
            {
                uint8_t val;
                rt_device_open(device, RT_DEVICE_OFLAG_RDONLY);
                if (1 == rt_device_read(device, 0, &val, 1))
                {
                    LOG_I("'lcdlight' device brightness level=%d", val);
                }
                else
                {
                    LOG_E("Read 'lcdlight' device brightness level fail");
                }
                rt_device_close(device);
            }
            else
            {
                LOG_I("No 'lcdlight' device found.");
            }

            LOG_I("drv_lcd record brightness level=%d", drv_lcd.brightness);
        }
    }
    else if (strcmp(argv[1], "rotate") == 0)
    {
        if (argc > 2)
        {
            uint16_t angle = strtoul(argv[2], 0, 10);
            if (180 == angle)
                api_lcd_control(&drv_lcd.parent, RTGRAPHIC_CTRL_ROTATE_180, NULL);
        }
        else
        {
            DEBUG_PRINTF("lcd_ctrl rotate 180\n");
        }
    }
    else if (strcmp(argv[1], "log") == 0)
    {
        if (argc > 2)
        {
            uint16_t v = strtoul(argv[2], 0, 10);

            drv_lcd.send_time_log = (v == 0) ? 0 : 1;
        }
        DEBUG_PRINTF("log %d\n", drv_lcd.send_time_log);
    }
    else if (strcmp(argv[1], "freq") == 0)
    {
        if (argc > 2)
        {
            uint32_t freq = strtoul(argv[2], 0, 10);

            HAL_LCDC_SetFreq(&drv_lcd.hlcdc, freq);
        }
        DEBUG_PRINTF("freq %d\n", drv_lcd.hlcdc.Init.freq);
    }
    else if (strcmp(argv[1], "te") == 0)
    {
        if (argc > 2)
        {
            uint32_t te = strtoul(argv[2], 0, 10);

            HAL_LCDC_Enable_TE(&drv_lcd.hlcdc, te);
            DEBUG_PRINTF("te= %d\n", te);
        }
    }
    else if (strcmp(argv[1], "assert") == 0)
    {
        if (argc > 2)
        {
            uint32_t v = strtoul(argv[2], 0, 10);

            drv_lcd.assert_timeout = v;
            DEBUG_PRINTF("assert= %d\n", v);
        }
    }




#if 0
    else if (strcmp(argv[1], "swrd") == 0)
    {
        if (argc > 2)
        {
            uint16_t v = strtoul(argv[2], 0, 10);

            drv_lcd.hlcdc.Init.cfg.spi.readback_from_Dx = v;
        }
        DEBUG_PRINTF("readback_from_Dx %d\n", drv_lcd.hlcdc.Init.cfg.spi.readback_from_Dx);
    }
    else if (strcmp(argv[1], "auto_lp") == 0)
    {
        if (argc > 2)
        {
            uint16_t v = strtoul(argv[2], 0, 10);
            drv_lcd.auto_lowpower = v;
            DEBUG_PRINTF("lcd auto_lowpower =%d", v);
        }
    }
    else if (strcmp(argv[1], "cmprate") == 0)
    {
        uint32_t cmprate;
        if (argc > 2)
        {
            cmprate = strtoul(argv[2], 0, 10);
            api_lcd_control(&drv_lcd.parent, SF_GRAPHIC_CTRL_LCDC_LAYER_BUF_CMPR_RATE, &cmprate);
            DEBUG_PRINTF("lcd comapre rate is %d.\n", cmprate);
        }
    }
    else if (strcmp(argv[1], "set_window") == 0)
    {
        if (argc > 5)
        {
            uint16_t x0 = strtoul(argv[2], 0, 10);
            uint16_t x1 = strtoul(argv[3], 0, 10);
            uint16_t y0 = strtoul(argv[4], 0, 10);
            uint16_t y1 = strtoul(argv[5], 0, 10);
            DEBUG_PRINTF("set_windown [x0x1:%d,%d,y0y1:%d,%d]", x0, x1, y0, y1);

            api_lcd_set_window(x0,  y0, x1, y1);

        }
    }
    else if (strcmp(argv[1], "draw_layer") == 0)
    {
        if (argc > 3)
        {
            LCDC_LayerCfgTypeDef *p_layer = &drv_lcd.hlcdc.Layer[drv_lcd.select_layer];
            uint16_t x0 = strtoul(argv[2], 0, 10);
            uint16_t y0 = strtoul(argv[3], 0, 10);
            uint16_t x1 = x0 + (p_layer->data_area.x1 - p_layer->data_area.x0); //Keep layer size
            uint16_t y1 = y0 + (p_layer->data_area.y1 - p_layer->data_area.y0);

            DEBUG_PRINTF("draw_layer 0x%x [x0x1:%d,%d,y0y1:%d,%d]", p_layer->data, x0, x1, y0, y1);
            api_lcd_draw_rect_async((const char *) p_layer->data, x0, y0, x1, y1);


        }
    }
#endif /* 0 */



    return 0;
}
MSH_CMD_EXPORT(lcd_ctrl, lcd control);
#endif /* RT_USING_FINSH */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
