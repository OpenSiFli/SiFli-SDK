#if 1//def DRV_LCD_TEST

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_lcd.h"
#include "drv_io.h"
#include "drv_common.h"
#include "drv_lcd_private.h"
#include "drv_ext_dma.h"
#include "mem_section.h"
#include "string.h"
#include <stdlib.h>


#define  DBG_LEVEL            DBG_LOG //DBG_ERROR  //

#define LOG_TAG                ">>>drv.lcd.test"
#include "log.h"

static struct rt_device_graphic_info lcd_info;
static rt_sem_t p_async_draw_sema = NULL;
static rt_err_t (*ancestor_tx_complete)(rt_device_t dev, void *buffer);
static uint32_t loop_cnt;

static uint32_t loop_end(void)
{
    uint32_t retv;
    rt_enter_critical();

    if (loop_cnt > 0)
    {
        retv = loop_cnt;

        loop_cnt--;
    }
    else
    {
        retv = 0;
    }

    rt_exit_critical();
    return retv;
}
static uint32_t rand_in_range(uint32_t min, uint32_t max)
{
    return  min + (rand() % (max - min + 1));
}
static rt_err_t lcd_flush_done(rt_device_t dev, void *buffer)
{

    if (ancestor_tx_complete) ancestor_tx_complete(dev, buffer);

    if (p_async_draw_sema) rt_sem_release(p_async_draw_sema);
    return RT_EOK;
}

static void set_brightness(rt_device_t lcd_device)
{
    rt_err_t err = rt_device_open(lcd_device, RT_DEVICE_OFLAG_RDWR);
    if ((RT_EOK == err) || (-RT_EBUSY == err))
    {
        uint8_t brightness = (uint8_t) rand_in_range(50, 100);
        rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BRIGHTNESS, &brightness);

        if (RT_EOK == err) rt_device_close(lcd_device);
    }
}


static void draw(rt_device_t lcd_device, uint8_t async_draw, uint8_t random_area)
{
    int32_t dx, dy, w, h;

    const char *frambuffer = (const char *)HPSYS_RAM0_BASE;
#ifdef BSP_USING_RAMLESS_LCD
    random_area = 0; //Only full screen FB supported on RAMLESS screen
#endif /* BSP_USING_RAMLESS_LCD */
    if (random_area)
    {
        dx = (int32_t) rand_in_range(0, lcd_info.width - lcd_info.draw_align);
        dy = (int32_t) rand_in_range(0, lcd_info.height - lcd_info.draw_align);

        w  = (int32_t) rand_in_range(lcd_info.draw_align, lcd_info.width - dx);
        h  = (int32_t) rand_in_range(lcd_info.draw_align, lcd_info.height - dy);

        /*In case of some LCD need align to 2 or 4 pixel*/
        dx = RT_ALIGN_DOWN(dx, lcd_info.draw_align);
        dy = RT_ALIGN_DOWN(dy, lcd_info.draw_align);

        w  = RT_ALIGN_DOWN(w, lcd_info.draw_align);
        h  = RT_ALIGN_DOWN(h, lcd_info.draw_align);
    }
    else
    {
        //Draw full screen
        dx = 0;
        dy = 0;
        w = lcd_info.width;
        h = lcd_info.height;
    }

    if (w && h)
    {
        rt_graphix_ops(lcd_device)->set_window(dx, dy, dx + w - 1, dy + h - 1);

        set_brightness(lcd_device);

        if (async_draw)
        {
            if (p_async_draw_sema)
            {
                rt_sem_control(p_async_draw_sema, RT_IPC_CMD_RESET, (void *)1);
                rt_sem_take(p_async_draw_sema, RT_WAITING_FOREVER);
            }
            rt_graphix_ops(lcd_device)->draw_rect_async((const char *)frambuffer, dx, dy, dx + w - 1, dy + h - 1);
        }
        else
            rt_graphix_ops(lcd_device)->draw_rect((const char *)frambuffer, dx, dy, dx + w - 1, dy + h - 1);
    }
}

static void standby_sleep_wakeup(rt_device_t lcd_device)
{
    rt_device_control(lcd_device, RT_DEVICE_CTRL_SUSPEND, NULL);
    HAL_RCC_ResetModule(RCC_MOD_LCDC1);  //Reset LCDC DSI DSI-PHY etc. to act like entry real standby
#ifdef HAL_DSI_MODULE_ENABLED
    HAL_RCC_ResetModule(RCC_MOD_DSIHOST);
    HAL_RCC_ResetModule(RCC_MOD_DSIPHY);
#endif /* HAL_DSI_MODULE_ENABLED */
    rt_device_control(lcd_device, RT_DEVICE_CTRL_RESUME, NULL);
}

static void brightness_set_task(void *parameter)
{
    rt_device_t device = (rt_device_t) parameter;

    while (loop_end())
    {
        set_brightness(device);

        rt_thread_mdelay((rt_int32_t)rand_in_range(16, 100));
    }
}

static void async_draw_task(void *parameter)
{
    rt_device_t lcd_device = (rt_device_t) parameter;

    while (loop_end())
    {
        draw(lcd_device, 1, 1);
        rt_thread_mdelay((rt_int32_t)rand_in_range(16, 100));

    }
}

static void sync_draw_task(void *parameter)
{
    rt_device_t lcd_device = (rt_device_t) parameter;

    while (loop_end())
    {
        draw(lcd_device, 0, 1);
        rt_thread_mdelay((rt_int32_t)rand_in_range(16, 100));

    }
}


static void draw_close_open_task(void *parameter)
{
    rt_device_t lcd_device = (rt_device_t) parameter;
    bool lcd_drawing;
    uint8_t idle_mode_on;

    while (loop_end())
    {
        rt_device_control(lcd_device, RTGRAPHIC_CTRL_GET_BUSY, &lcd_drawing);
        if (!lcd_drawing)
        {

            //rt_thread_mdelay((rt_int32_t)rand_in_range(1, 100));
            rt_device_close(lcd_device);
            //rt_thread_mdelay((rt_int32_t)rand_in_range(1, 100));
            RT_ASSERT(RT_EOK == rt_device_open(lcd_device, RT_DEVICE_OFLAG_RDWR));

            //rt_thread_mdelay((rt_int32_t)rand_in_range(1, 100));


            uint16_t framebuffer_color_format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
            rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &framebuffer_color_format);
        }
        draw(lcd_device, 0, 1);
        rt_thread_mdelay(30);//Make sure to see what we drawn
    }
}


static void draw_onoff_sleep_task(void *parameter)
{
    rt_device_t lcd_device = (rt_device_t) parameter;
    bool lcd_drawing;
    uint8_t idle_mode_on;

    while (loop_end())
    {
        rt_device_control(lcd_device, RTGRAPHIC_CTRL_GET_BUSY, &lcd_drawing);
        if (!lcd_drawing)
        {

            rt_device_control(lcd_device, RTGRAPHIC_CTRL_POWEROFF, NULL);
            standby_sleep_wakeup(lcd_device);
            rt_device_control(lcd_device, RTGRAPHIC_CTRL_POWERON, NULL);
        }
        draw(lcd_device, 0, 1);
        rt_thread_mdelay(30);
    }
}

#if defined(PKG_USING_LITTLEVGL2RTT) && defined(LV_FRAME_BUF_SCHEME_0)
static void extdma_copy_task(void *parameter)
{
    rt_device_t lcd_device = (rt_device_t) parameter;
    bool lcd_drawing;
    uint8_t idle_mode_on;

    while (loop_end())
    {
        draw(lcd_device, 1, 0);
        do
        {
            rt_device_control(lcd_device, RTGRAPHIC_CTRL_GET_BUSY, &lcd_drawing);
        }
        while (lcd_drawing);


        {
            rt_device_control(lcd_device, RTGRAPHIC_CTRL_POWEROFF, NULL);
            rt_thread_mdelay(5);

            standby_sleep_wakeup(lcd_device);
            rt_device_control(lcd_device, RTGRAPHIC_CTRL_POWERON, NULL);
        }
        draw(lcd_device, 1, 0);
        rt_thread_mdelay(30);
    }
}
#endif

static void draw_idle_sleep_task(void *parameter)
{
    rt_device_t lcd_device = (rt_device_t) parameter;
    bool lcd_drawing;
    uint8_t idle_mode_on;

    while (loop_end())
    {
        rt_device_control(lcd_device, RTGRAPHIC_CTRL_GET_BUSY, &lcd_drawing);
        if (!lcd_drawing)
        {

            idle_mode_on = 1;
            rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_MODE, (void *)&idle_mode_on);


            standby_sleep_wakeup(lcd_device);

            idle_mode_on = 0;
            rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_MODE, (void *)&idle_mode_on);

        }
        draw(lcd_device, 0, 1);
    }
}

static void draw_suspend_resume_task(void *parameter)
{
    rt_device_t lcd_device = (rt_device_t) parameter;
    bool lcd_drawing;
    uint8_t idle_mode_on;

    while (loop_end())
    {
        rt_device_control(lcd_device, RTGRAPHIC_CTRL_GET_BUSY, &lcd_drawing);
        if (!lcd_drawing)
        {
            standby_sleep_wakeup(lcd_device);
        }
        draw(lcd_device, 0, 1);
    }
}

static void hot_plug_in_out_task(void *parameter)
{
    rt_device_t lcd_device = (rt_device_t) parameter;
    uint8_t idle_mode_on;

    while (1)
    {
        bool lcd_drawing;
        rt_device_control(lcd_device, RTGRAPHIC_CTRL_GET_BUSY, &lcd_drawing);
        if (!lcd_drawing) break;
    }
    uint16_t framebuffer_color_format = RTGRAPHIC_PIXEL_FORMAT_RGB565;


    while (loop_end())
    {
        for (uint32_t plug_inout_type = 0; plug_inout_type < 2; plug_inout_type++)
            for (uint32_t resume_method = 0; resume_method < 2; resume_method++)
            {
                LOG_I("\n");
                LOG_I("A.Plug out LCD. type(%d).", plug_inout_type);
                if (0 == plug_inout_type)
                {
                    /*Read ID avaiable, & Plug out state*/
                    rt_device_close(lcd_device);
                    rt_device_control(lcd_device, SF_GRAPHIC_CTRL_LCD_PRESENT, (void *)0);
                    RT_ASSERT(RT_EOK == rt_device_open(lcd_device, RT_DEVICE_OFLAG_RDWR));
                    rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &framebuffer_color_format);

                }
                else if (1 == plug_inout_type)
                {
                    /*Read ID unaviable, & Plug out state*/
                    rt_device_control(lcd_device, SF_GRAPHIC_CTRL_ASSERT_IF_DRAWTIMEOUT, (void *)0);
                    BSP_LCD_Reset(0);//Pull down LCD RESET to pretend LCD was plug out
                }

                LOG_I("\n");
                LOG_I("B. Execute LCD at plug out state. type(%d)", plug_inout_type);
                set_brightness(lcd_device);
                draw(lcd_device, 0, 0);
                rt_thread_mdelay(1000);//Make sure to see what we drawn

                LOG_I("\n");
                LOG_I("C. Plug in LCD. type(%d)", plug_inout_type);
                if (0 == plug_inout_type)
                {
                    rt_device_control(lcd_device, SF_GRAPHIC_CTRL_LCD_PRESENT, (void *)1);
                }
                else if (1 == plug_inout_type)
                {
                    rt_device_control(lcd_device, SF_GRAPHIC_CTRL_ASSERT_IF_DRAWTIMEOUT, (void *)1);
                    BSP_LCD_Reset(1);
                }

                /*D. Resume LCD*/
                LOG_I("\n");
                if (0 == resume_method)
                {
                    LOG_I("D. Resume by device close&open");
                    rt_device_close(lcd_device);
                    RT_ASSERT(RT_EOK == rt_device_open(lcd_device, RT_DEVICE_OFLAG_RDWR));
                    rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &framebuffer_color_format);

                }
                else if (1 == resume_method)
                {
                    LOG_I("D. Resume by POWER OFF&ON");
                    rt_device_control(lcd_device, RTGRAPHIC_CTRL_POWEROFF, NULL);
                    standby_sleep_wakeup(lcd_device);
                    rt_device_control(lcd_device, RTGRAPHIC_CTRL_POWERON, NULL);
                }

                /*E. Execute LCD at plug in state*/
                LOG_I("\n");
                LOG_I("E. Draw LCD at plug in state(type%d, method %d).", plug_inout_type, resume_method);
                set_brightness(lcd_device);
                draw(lcd_device, 0, 0);
                rt_thread_mdelay(1000);//Make sure to see what we drawn
            }
    }
}



static rt_err_t drv_lcd_test(int argc, char **argv)
{
    rt_device_t lcd_device = rt_device_find("lcd");

    if (argc < 2)
    {
        LOG_E("drv_lcd_test <loop_cnt> <test_group>");
        return RT_EOK;
    }

    if (lcd_device)
    {
        rt_err_t result = rt_device_open(lcd_device, RT_DEVICE_OFLAG_RDWR);

        if (1) // (rt_device_open(lcd_device, RT_DEVICE_OFLAG_RDWR) == RT_EOK)
        {


            uint32_t test_group;
            loop_cnt = strtoul(argv[1], 0, 10);
            test_group = strtoul(argv[2], 0, 10);

            ancestor_tx_complete = lcd_device->tx_complete;
            p_async_draw_sema = rt_sem_create("lcdsem", 1, RT_IPC_FLAG_FIFO);
            rt_device_set_tx_complete(lcd_device, lcd_flush_done);
            if (rt_device_control(lcd_device, RTGRAPHIC_CTRL_GET_INFO, &lcd_info) == RT_EOK)
            {
                LOG_I("Lcd info w:%d, h%d, bits_per_pixel %d\r\n", lcd_info.width, lcd_info.height, lcd_info.bits_per_pixel);
            }

            uint16_t cf;
            if (16 == lcd_info.bits_per_pixel)
                cf = RTGRAPHIC_PIXEL_FORMAT_RGB565;
            else if (24 == lcd_info.bits_per_pixel)
                cf = RTGRAPHIC_PIXEL_FORMAT_RGB888;
            else
                RT_ASSERT(0);

            rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &cf);




            if (0 == test_group)
            {
                rt_thread_t t_brightness = rt_thread_create("lcdt_a", brightness_set_task, lcd_device, 1024, RT_THREAD_PRIORITY_MIDDLE, 1);
                rt_thread_startup(t_brightness);

                rt_thread_t t_draw_sync = rt_thread_create("lcdt_b", sync_draw_task, lcd_device, 1024, RT_THREAD_PRIORITY_MIDDLE, 1);
                rt_thread_startup(t_draw_sync);

                rt_thread_t t_draw_async = rt_thread_create("lcdt_c", async_draw_task, lcd_device, 1024, RT_THREAD_PRIORITY_MIDDLE, 1);
                rt_thread_startup(t_draw_async);
            }
            else if (1 == test_group)
            {
                rt_thread_t t_draw_close_open = rt_thread_create("lcdt_d", draw_close_open_task, lcd_device, 1024, RT_THREAD_PRIORITY_MIDDLE, 1);
                rt_thread_startup(t_draw_close_open);
            }
            else if (2 == test_group)
            {
                rt_thread_t t_draw_onoff_sleep = rt_thread_create("lcdt_e", draw_onoff_sleep_task, lcd_device, 1024, RT_THREAD_PRIORITY_MIDDLE, 1);
                rt_thread_startup(t_draw_onoff_sleep);
            }
            else if (3 == test_group)
            {
                rt_thread_t t_draw_idle_sleep = rt_thread_create("lcdt_f", draw_idle_sleep_task, lcd_device, 1024, RT_THREAD_PRIORITY_MIDDLE, 1);
                rt_thread_startup(t_draw_idle_sleep);
            }
            else if (4 == test_group)
            {
                rt_thread_t t_draw_supend_resume = rt_thread_create("lcdt_g", draw_suspend_resume_task, lcd_device, 1024, RT_THREAD_PRIORITY_MIDDLE, 1);
                rt_thread_startup(t_draw_supend_resume);
            }
            else if (5 == test_group)
            {
                rt_thread_t t_hot_plug_in_out = rt_thread_create("lcdt_h", hot_plug_in_out_task, lcd_device, 1024, RT_THREAD_PRIORITY_MIDDLE, 1);
                rt_thread_startup(t_hot_plug_in_out);
            }
#if defined(PKG_USING_LITTLEVGL2RTT) && defined(LV_FRAME_BUF_SCHEME_0)
            else if (6 == test_group)
            {
                rt_thread_t t_extdma_copy = rt_thread_create("lcdt_i", extdma_copy_task, lcd_device, 1024, RT_THREAD_PRIORITY_MIDDLE, 1);
                rt_thread_startup(t_extdma_copy);
            }
#endif
            else
            {
                while (loop_end())
                {
                    ;//Nothing todo
                }
            }


            //Wait all test task exit
            while (loop_cnt != 0) rt_thread_mdelay(100);

            rt_thread_mdelay(1000);
            rt_device_set_tx_complete(lcd_device, ancestor_tx_complete);

            if (RT_EOK == result) rt_device_close(lcd_device);
            lcd_device = RT_NULL;


            rt_sem_delete(p_async_draw_sema);
            p_async_draw_sema = NULL;

        }
        else
        {
            LOG_I("Lcd open error!\n");
        }
    }
    else
    {
        LOG_I("Can't find lcd");
    }
    return RT_EOK;
}
MSH_CMD_EXPORT(drv_lcd_test, drv lcd test);



#endif /* DRV_LCD_TEST */

