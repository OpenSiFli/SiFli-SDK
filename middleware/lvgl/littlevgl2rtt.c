#include <rtthread.h>
#include "littlevgl2rtt.h"
#ifdef LV_USE_LVSF
    #include "lv_ext_resource_manager.h"
#endif /* LV_USE_LVSF */
#include "cpu_usage_profiler.h"
#if !defined(_MSC_VER)
    #include "bf0_hal.h"
#endif

#define LOG_TAG  "LVGL"

#include "log.h"
#if defined(LV_USE_GPU)&&(1==LV_USE_GPU)
    #include "drv_epic.h"
#endif


#ifdef LCD_SDL2

float cpu_get_usage(void)
{
    return 0.2f;
}

#endif

static uint32_t frame_cnt = 0;
static float fps;
static float epic_perf;//Epic busy percentage
uint8_t fb_get_cmpr_rate(void);



#if LV_USE_LOG
    static void log_print(lv_log_level_t level, const char *file_name, uint32_t line, const char *func, const char *dsc);
#endif
#if defined(LV_USE_GPU)&&(1==LV_USE_GPU)
    extern EPIC_HandleTypeDef *drv_get_epic_handle(void);
#endif

static bool fps_statistics_first = true;

static lv_obj_t *fps_cpu_load_label = NULL;
static lv_draw_label_dsc_t lv_style_fps_cpu_load_label;
static lv_timer_t *perf_task = NULL;
#define FPS_LABEL_WIDTH 80
#define FPS_LABEL_HEIGHT (20 /*font height*/ * 5 /*lines*/)

#define FPS_LABEL_BUFFER_SIZE (FPS_LABEL_WIDTH * FPS_LABEL_HEIGHT * LV_IMG_PX_SIZE_ALPHA_BYTE)
#define FPS_STATISTICS_PERIOD_MS  2000
#define FPS_LOWEST_x_PERCENT  5
static bool fps_on = 0;
static uint8_t mem_on = 0;
#ifdef FPS_LOWEST_x_PERCENT
    #define FPS_LOWEST_x_PERCENT_MAX  ((FPS_LOWEST_x_PERCENT * (FPS_STATISTICS_PERIOD_MS/LV_DISP_DEF_REFR_PERIOD) / 100) + 10/*Exra 10 more records*/)


    static bool lowest_fps_statistics_first = true;
    static uint32_t lowest_frames[FPS_LOWEST_x_PERCENT_MAX]; //The lowest frames refresh times array, e.g. {200, 150, 100, ...}
    static uint16_t lowest_frames_count; //The valid items numbers of 'lowest_frames' array.
    static float lowest_frames_fps_avg;
#endif /* FPS_LOWEST_x_PERCENT */


extern void list_mem(void);
extern void list_memheap(void);


#ifdef SF32LB55X
    #define MONITOR_IRANGE (CACHE_CCR_IRANGE_QSPI1_4 | CACHE_CCR_IRANGE_QSPI2)
    #define MONITOR_DRANGE (CACHE_CCR_DRANGE_QSPI1_4 | CACHE_CCR_DRANGE_QSPI2)
#elif defined(SF32LB58X)
    #define MONITOR_IRANGE (CACHE_CCR_IRANGE_MPI1 | CACHE_CCR_IRANGE_MPI4)
    #define MONITOR_DRANGE (CACHE_CCR_DRANGE_MPI1 | CACHE_CCR_DRANGE_MPI4)

#elif defined(BSP_ENABLE_MPI1)
    #define MONITOR_IRANGE (CACHE_CCR_IRANGE_MPI1)
    #define MONITOR_DRANGE (CACHE_CCR_DRANGE_MPI1)
#elif defined(BSP_ENABLE_MPI2)
    #define MONITOR_IRANGE (CACHE_CCR_IRANGE_MPI2)
    #define MONITOR_DRANGE (CACHE_CCR_DRANGE_MPI2)
#elif !defined(BSP_USING_PC_SIMULATOR)
    #error Please check monitor range.
#endif


int display_fps_onoff(void)
{
    set_display_fps_and_cpu_load(!fps_on);
    rt_kprintf("fps_on: %d\n", fps_on);
    return 0;
}

#ifdef RT_USING_FINSH
MSH_CMD_EXPORT_ALIAS(display_fps_onoff, fps2, fps2: open or close fps log);
#endif /* RT_USING_FINSH */

int mem_disp_onoff(void)
{
    mem_on = (mem_on + 1) & 0x01;
    rt_kprintf("mem_disp: %d\n", mem_on);
    return 0;
}
#ifdef RT_USING_FINSH
MSH_CMD_EXPORT_ALIAS(mem_disp_onoff, mem_usage, mem_usage: open or close memory usage log);
#endif /* RT_USING_FINSH */



void display_fps_and_cpu_load(void)
{
    float perf_fps = lv_get_fps();

    uint8_t perf_lvgl = 100 - lv_timer_get_idle();

    float perf_cpu = cpu_get_usage();

    //Force turn on fps
    //if(!get_display_fps_and_cpu_load()) display_fps_onoff();

#ifdef RT_USING_FINSH
    extern uint32_t used_sram_size(void);
    static uint32_t pre_sram_size;

    if (fps_on)
    {
#if  !defined(_MSC_VER)
        float irate, drate;
        HAL_CACHE_GetMissRate(&irate, &drate, 1);
        LOG_I("fps %.1f cpu %.1f EPIC %.1f LVGL:%d irate:%.1f, drate:%.1f", perf_fps, perf_cpu, epic_perf, perf_lvgl, irate, drate);
#else
        LOG_I("fps %.1f cpu %.1f LVGL:%d", perf_fps, perf_cpu, perf_lvgl);
#endif

#ifdef FPS_LOWEST_x_PERCENT
        LOG_I("%d%% lowest frame fps avg= %.1f", FPS_LOWEST_x_PERCENT, lowest_frames_fps_avg);
#endif /* FPS_LOWEST_x_PERCENT */

    }

    if (mem_on && pre_sram_size != used_sram_size())
    {
        list_mem();
        list_memheap();
        pre_sram_size  = used_sram_size();
    }
#endif

    char buff[100];
    /*
    rt_sprintf(buff, "F:%.1f\nU:%.1f\nI:%d\nC:%d\n", perf_fps, perf_cpu, lvgl_idle,
    #if LV_USE_GPU
               fb_get_cmpr_rate()
    #else
               0
    #endif
              );
    */
#ifdef FPS_LOWEST_x_PERCENT
    rt_sprintf(buff, "F:%.0f  %d%%:%.0f  C:%.0f  G:%.0f  LV:%d", perf_fps,
               FPS_LOWEST_x_PERCENT, lowest_frames_fps_avg,
               perf_cpu, epic_perf, perf_lvgl);

#else
    rt_sprintf(buff, "F:%.1f  C:%.1f  G:%.1f  LV:%d", perf_fps, perf_cpu, epic_perf, perf_lvgl);
#endif /* FPS_LOWEST_x_PERCENT */

    if (fps_cpu_load_label)
    {
        lv_label_set_text(fps_cpu_load_label, buff);
        lv_obj_invalidate(fps_cpu_load_label);
    }

}

static uint32_t get_elapsed_tick(uint32_t prev_time)
{
    uint32_t curr_time, elapsed_time;

    curr_time = rt_tick_get();

    if (curr_time >= prev_time)
    {
        elapsed_time = curr_time - prev_time;
    }
    else
    {
        elapsed_time = RT_UINT32_MAX - prev_time + 1 + curr_time;
    }

    return elapsed_time;
}

static void perf_task_handler(lv_timer_t *task)
{
    static uint32_t prev_time = 0;
    uint32_t elapsed_time;
#if defined(LV_USE_GPU)&&(1==LV_USE_GPU)
    static uint32_t prev_epic_cnt = 0;
#endif


    if (fps_statistics_first)
    {
        prev_time = rt_tick_get();
#if defined(LV_USE_GPU)&&(1==LV_USE_GPU)
        prev_epic_cnt = drv_get_epic_handle()->PerfCnt;
#endif
        fps_statistics_first = false;
        return;
    }

    elapsed_time = get_elapsed_tick(prev_time);

    if (elapsed_time > rt_tick_from_millisecond(FPS_STATISTICS_PERIOD_MS))
    {
#ifdef FPS_LOWEST_x_PERCENT
        uint16_t calc_items = frame_cnt * FPS_LOWEST_x_PERCENT / 100;

        //for(int16_t i = 0; i < lowest_frames_count; i++) LOG_I("%02d: %03d", i, lowest_frames[i]);


        LOG_I("\n%d%% lowest frame(ms):", FPS_LOWEST_x_PERCENT);
        if (calc_items > lowest_frames_count)
        {
            LOG_I("Need more data: expect %d, got %d.", calc_items, lowest_frames_count);
            lowest_frames_fps_avg = 0.0f;
        }
        else
        {
            uint32_t lowest_frames_sum = 0;
            for (int16_t i = 0; i < calc_items; i++)
            {
                lowest_frames_sum += lowest_frames[i];
                LOG_I("%02d: %03d", i, lowest_frames[i]);
            }

            lowest_frames_fps_avg = 1000.0f / ((float)lowest_frames_sum / calc_items);
        }

        lowest_frames_count = 0;
#endif /* FPS_LOWEST_x_PERCENT */

        fps = (float)frame_cnt * RT_TICK_PER_SECOND / elapsed_time;
        frame_cnt = 0;
        prev_time = rt_tick_get();

#if defined(LV_USE_GPU)&&(1==LV_USE_GPU)
        {
            uint32_t cost_cnt;

            {
                uint32_t cur_epic_cnt = drv_get_epic_handle()->PerfCnt;
                if (cur_epic_cnt >= prev_epic_cnt)
                    cost_cnt = cur_epic_cnt - prev_epic_cnt;
                else
                    cost_cnt = cur_epic_cnt + (UINT32_MAX - prev_epic_cnt + 1);

                prev_epic_cnt = cur_epic_cnt;
            }

            uint32_t epic_cost_ms = cost_cnt / (HAL_RCC_GetHCLKFreq(CORE_ID_HCPU) / 1000);
            uint32_t elapsed_ms  = elapsed_time * 1000 / RT_TICK_PER_SECOND;

            epic_perf = (float)(100 * epic_cost_ms) / elapsed_ms;
        }
#else
        epic_perf = 0;
#endif

        display_fps_and_cpu_load();
    }
}


void set_display_fps_and_cpu_load(int en)
{
#if !defined(_MSC_VER)
    if (en)
        HAL_CACHE_Enable(MONITOR_IRANGE, MONITOR_DRANGE);
    else
        HAL_CACHE_Disable();
#endif

#ifdef FPS_LOWEST_x_PERCENT
    lowest_frames_count = 0;
    lowest_frames_fps_avg = 0.0f;
    lowest_fps_statistics_first = true;
#endif /* FPS_LOWEST_x_PERCENT */
    frame_cnt = 0;
    fps_statistics_first = true;

    fps_on = en;
}

int get_display_fps_and_cpu_load(void)
{
    return fps_on;
}

static void enable_fps_label(int en)
{
    if ((NULL == fps_cpu_load_label) && en)
    {
        lv_obj_t *sys_scr = lv_disp_get_layer_sys(NULL);      /*Get the current screen*/
        fps_cpu_load_label = lv_label_create(sys_scr);
#ifdef LV_USE_LVSF
        lv_ext_set_local_font(fps_cpu_load_label, FONT_NORMAL, lv_palette_main(LV_PALETTE_RED));
#endif /* LV_USE_LVSF */
        lv_obj_align(fps_cpu_load_label, LV_ALIGN_TOP_MID, 0, 0);
        //display_fps_and_cpu_load();
        lv_label_set_text(fps_cpu_load_label, "Waitting...");
        perf_task = lv_timer_create(perf_task_handler, 500, (void *)0);
    }
    else if ((fps_cpu_load_label) && (!en))
    {
        lv_obj_del(fps_cpu_load_label);
        fps_cpu_load_label = NULL;

        lv_timer_del(perf_task);
        perf_task = NULL;
    }
    else
    {
        //Ignore repeated setup.
    }
}
#ifndef DISABLE_LVGL_V8


void perf_monitor(struct _lv_disp_drv_t *disp_drv, uint32_t time, uint32_t px)
{
    if (fps_on)
    {
        frame_cnt++;

#ifdef FPS_LOWEST_x_PERCENT
        static uint32_t prev_time = 0;
        if (lowest_fps_statistics_first)
        {
            lowest_fps_statistics_first = false;
        }
        else
        {
            uint32_t elapsed_time = get_elapsed_tick(prev_time);


            //Avoid the average of lowest frame larger than 'fps'
            if (elapsed_time < LV_DISP_DEF_REFR_PERIOD) elapsed_time = LV_DISP_DEF_REFR_PERIOD;

            if (0 == lowest_frames_count)
            {
                lowest_frames[0] = elapsed_time;//The first item
            }
            else
            {
                int16_t i;
                for (i = lowest_frames_count - 1; i >= 0 ; i--)
                {
                    if (elapsed_time <= lowest_frames[i])
                    {
                        i++; //Replace 'lowest_frames[i]' with 'time'
                        break;
                    }
                }

                if (i == lowest_frames_count) //New item
                {
                    if (i < FPS_LOWEST_x_PERCENT_MAX)   lowest_frames[i] = elapsed_time;
                }
                else
                {
                    int16_t j = LV_MIN(FPS_LOWEST_x_PERCENT_MAX - 1, lowest_frames_count);

                    while ((j > i) && (j > 0))
                    {
                        lowest_frames[j] = lowest_frames[j - 1];
                        j--;
                    }

                    if (i > 0)
                        lowest_frames[i] = elapsed_time;
                    else
                        lowest_frames[0] = elapsed_time;
                }
            }

            if (lowest_frames_count < FPS_LOWEST_x_PERCENT_MAX) lowest_frames_count++;
        }

        prev_time = rt_tick_get();
#endif /* FPS_LOWEST_x_PERCENT */
    }

    enable_fps_label(fps_on);
}
#endif /* DISABLE_LVGL_V8 */

float lv_get_fps(void)
{
    return fps;
}

#if 0//LV_USE_LOG
static void log_print(lv_log_level_t level, const char *file_name, uint32_t line, const char *func, const char *dsc)
{

    LOG_D("%s \t(func:%s, line:#%d)\n", dsc, func, line);
    if (level >= LV_LOG_LEVEL_ERROR)
    {
        RT_ASSERT(0 && "LV_ASSERT");
    }

}

#endif

#if LV_USE_LOG
#ifndef DISABLE_LVGL_V8
static void lv_rt_log(const char *buf)
{
    LOG_RAW(buf);
}
#else
static void lv_rt_log(lv_log_level_t level, const char *buf)
{
    LOG_RAW(buf);
}
#endif /* DISABLE_LVGL_V8 */

#endif

int gui_lib_init(void)
{

#if LV_USING_FREETYPE_ENGINE
    extern uint32_t ft_get_cache_size(void);
    lvsf_font_load(ft_get_cache_size());
    /* open freetype */
    lv_freetype_open_font(false);

#endif




    lv_init();
#if LV_USE_TINY_TTF
    extern void lv_font_tiny_init(void);
    lv_font_tiny_init();
#endif
#if LV_USE_LOG
    lv_log_register_print_cb(lv_rt_log);
#endif
    return 0;
}
INIT_COMPONENT_EXPORT(gui_lib_init);

static rt_thread_t host_thread = NULL;
rt_thread_t lvgl_host_thread(void)
{
    return host_thread;
}

extern void lv_hal_init(const char *name);
rt_err_t littlevgl2rtt_init(const char *name)
{
    //lv_init();

    lv_hal_init(name);
#ifdef LV_USE_LVSF
    lv_theme_1_init();
#endif /* LV_USE_LVSF */
#if 0
#ifdef LCD_SDL2
    // Start the tick thread.
    {
        rt_thread_t thread = RT_NULL;

        /* littlevGL tick thread */
        thread = rt_thread_create("lv_tick", lvgl_tick_run, RT_NULL, 512, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
        if (thread == RT_NULL)
        {
            return RT_ERROR;
        }
        rt_thread_startup(thread);
    }
#endif
#endif


    host_thread = rt_thread_self();
    LOG_I("[littlevgl2rtt] Welcome to the littlevgl2rtt lib.");

    return RT_EOK;
}

/*#ifdef FINSH_USING_MSH
#include <finsh.h>

static rt_err_t switch_fps_cpu_label(int argc, char **argv)
{
    set_display_fps_and_cpu_load(!get_display_fps_and_cpu_load());

    return RT_EOK;
}
FINSH_FUNCTION_EXPORT(switch_fps_cpu_label, switch fps cpu label on display);
MSH_CMD_EXPORT(switch_fps_cpu_label, switch fps cpu label on display);
#endif*/

