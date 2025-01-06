/**
  ******************************************************************************
  * @file   drv_epic.c
  * @author Sifli software development team
  * @brief Common functions for BSP driver
  * @{
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
#include "drv_epic.h"
#ifdef BSP_USING_EPIC
#include <rthw.h>
#include "string.h"
#include "mem_section.h"
#ifdef HAL_EZIP_MODULE_ENABLED
    #include "drv_flash.h"
#endif


#ifdef _MSC_VER
    #define RETURN_ADDR ((rt_uint32_t) _ReturnAddress())
#else
    #define RETURN_ADDR ((rt_uint32_t) __builtin_return_address(0))
#endif


#define  DBG_LEVEL            DBG_INFO  //DBG_LOG //

#define LOG_TAG                "drv.epic"
#include "log.h"

#ifndef MIN
    #define MIN(x,y) (((x)<(y))?(x):(y))
#endif

#ifndef MAX
    #define MAX(x,y) (((x)>(y))?(x):(y))
#endif

#define TICK_TIME_START uint32_t __wait_time = rt_tick_get();
#define TICK_TIME_PRINT  LOG_E("Line %d, cost %d ms\r\n",__LINE__,rt_tick_get() - __wait_time);

#undef OPA_MAX
#define OPA_MAX 254

#ifdef RT_USING_PM
    #define GPU_DEVICE_START()   rt_pm_hw_device_start()
    #define GPU_DEVICE_STOP()     rt_pm_hw_device_stop()
#else
    #define GPU_DEVICE_START()
    #define GPU_DEVICE_STOP()

#endif  /* RT_USING_PM */

#define IS_DCACHED_RAM(addr) (((uint32_t) addr) >= (PSRAM_BASE))
#define GPU_BLEND_EXP_MS     500


#ifdef DRV_EPIC_NEW_API

#define render_list_pool_max 2
#define letter_pool_max 500
#define src_list_max 100

#define rl_flag_commit         0x01
#define rl_flag_rendering     0x02
#define rl_flag_overwritting     0x04

#define debug_rl_hist_num  src_list_max //Set '0' to disable history

typedef struct
{
    uint32_t  used;
    volatile uint32_t  flag;
    EPIC_LayerConfigTypeDef dst;


    volatile uint16_t src_list_len; /* Committed src len*/
    drv_epic_operation src_list[src_list_max];
    drv_epic_letter_type_t letter_pool[letter_pool_max];
    volatile uint32_t letter_pool_free;
    EPIC_AreaTypeDef  commit_area; /*Committed invalid area*/
} priv_render_list_t;


typedef struct
{

    priv_render_list_t *rl;
    uint16_t  src_idx;
    uint16_t  use_ram_epic;
    uint32_t  start_tick;
    uint32_t  cost_us;
} priv_render_hist_t;


typedef struct
{
    priv_render_list_t *rl;
    drv_epic_render_cb cb;
#if debug_rl_hist_num > 0
    priv_render_hist_t *p_hist;
#endif /* debug_rl_hist_num > 0 */
} epic_cbk_ctx_t;
#endif /*DRV_EPIC_NEW_API*/

typedef struct _split_render_t
{
    drv_epic_op_type_t  op;
    EPIC_AreaTypeDef dst_area;
    const uint8_t *dst_data;
    EPIC_AreaTypeDef render_area;       //Part of 'dst_area'
    EPIC_AreaTypeDef next_render_area;  //Part of 'render_area'


    /* For 'drv_epic_transform'*/
    EPIC_TransPath hor_path;
    EPIC_TransPath ver_path;
    void *user_data;
} split_render_t;

typedef struct
{
    EPIC_HandleTypeDef epic_handle;
    struct rt_semaphore epic_sema;

    EPIC_TypeDef RamEPIC;
#ifdef HAL_EZIP_MODULE_ENABLED
    EZIP_TypeDef RamEZIP;
    EZIP_HandleTypeDef ezip_handle;
#endif
#ifdef EPIC_DEBUG
    EPIC_OpHistTypeDef epic_op_hist;
#endif


#ifndef DRV_EPIC_NEW_API
    EPIC_LayerConfigTypeDef input_layers[MAX_EPIC_LAYER];
    uint8_t    input_layer_cnt;
    EPIC_LayerConfigTypeDef output_layer;
    EPIC_ColorDef grad_color[2][2];

    split_render_t split_rd;
    drv_epic_cplt_cbk cbk;

    bool cont_mode;

#else /*DRV_EPIC_NEW_API*/

    EPIC_HandleTypeDef epic_handle2; //Shadow handle
#ifdef HAL_EZIP_MODULE_ENABLED
    EZIP_HandleTypeDef ezip_handle2; //Shadow handle
#endif
    epic_cbk_ctx_t  epic_cb_ctx;


    priv_render_list_t render_list_pool[render_list_pool_max];
    priv_render_list_t *using_rl; //Committing rl
#if debug_rl_hist_num > 0
    uint32_t           hist_idx;
    priv_render_hist_t hist[debug_rl_hist_num];
#endif /* debug_rl_hist_num > 0 */

    EPIC_HandleTypeDef *using_epic;

    /*statistical*/
    uint32_t rd_count;
    uint32_t rd_min;
    uint32_t rd_max;
    uint32_t rd_total; /*ms*/
    uint32_t rd_lcd_total; /*ms*/
    uint32_t rd_epic_hw_us;
    uint32_t rd_epic_wait;
    uint32_t rd_flush_lcd_wait;

    struct rt_thread task;
    rt_mq_t  mq;
    struct rt_semaphore lcd_sema;
    uint8_t lcd_te;
    rt_err_t (*lcd_tx_complete)(rt_device_t dev, void *buffer);
#endif /* DRV_EPIC_NEW_API */

    uint32_t hclk_freq_Mhz;

    uint32_t gpu_last_op;
    uint32_t gpu_fg_addr;
    uint32_t gpu_bg_addr;
    uint32_t gpu_mask_addr;
    uint32_t gpu_output_addr;
    uint32_t gpu_output_size;
    uint32_t gpu_log_level;
    uint32_t gpu_timeout_cnt;

    uint32_t dbg_flag_dis_ram_instance: 1;
    uint32_t dbg_flag_print_rl : 1;
    uint32_t dbg_flag_reserved: 30;
    uint32_t dbg_flag_dis_merge_operations;
    uint32_t dbg_src_addr;
} EPIC_DrvTypeDef;


#define  epic_handle  drv_epic.epic_handle
#define  epic_sema    drv_epic.epic_sema

#define AreaString "x0y0x1y1=[%d,%d,%d,%d]"

#define PRINT_LAYER_INFO(layer, layer_name) \
                        LOG_D("%s,cf=0x%x,data=%x,total_w=%d,"AreaString" frac[%x,%x]", \
                                    (layer_name), (layer)->color_mode, (layer)->data, (layer)->total_width,\
                                    (layer)->x_offset, \
                                    (layer)->y_offset, \
                                    (layer)->x_offset + (layer)->width - 1, \
                                    (layer)->y_offset + (layer)->height - 1,\
                                    (layer)->x_offset_frac, (layer)->y_offset_frac\
                                    );

#define PRINT_LAYER_EXTRA_INFO(layer) \
                        LOG_D("ax=%d[0:NONE,1:MASK], alpha=%d, angle=%d, scale_xy=%d,%d, pivot_xy=%d,%d  mirror_hv=%d,%d", \
                                    (layer)->ax_mode, (layer)->alpha, \
                                    (layer)->transform_cfg.angle, \
                                    (layer)->transform_cfg.scale_x, (layer)->transform_cfg.scale_y, \
                                    (layer)->transform_cfg.pivot_x, (layer)->transform_cfg.pivot_y, \
                                    (layer)->transform_cfg.h_mirror, (layer)->transform_cfg.v_mirror\
                                    );


#define AreaParams(area) (area)->x0,(area)->y0,(area)->x1,(area)->y1
#define PRINT_AREA_INFO(area, area_name) LOG_D("%s "AreaString, area_name, AreaParams(area))


static EPIC_HandleTypeDef *epic = NULL;

L1_RET_BSS_SECT_BEGIN(drv_epic_ram)
static void dummy_function_for_source_insight_symbol_list0(void)
{
}
static EPIC_DrvTypeDef drv_epic;
static uint8_t drv_epic_inited = 0;
L1_RET_BSS_SECT_END

L1_NON_RET_BSS_SECT_BEGIN(drv_epic_stack)
L1_NON_RET_BSS_SECT(drv_epic_stack, ALIGN(RT_ALIGN_SIZE) static uint8_t drv_epic_stack[2048]);
L1_NON_RET_BSS_SECT_END



static void dummy_function_for_source_insight_symbol_list1(void)
{
}

static void print_gpu_error_info(void);




#ifdef HAL_EZIP_MODULE_ENABLED
__ROM_USED EZIP_HandleTypeDef *drv_get_ezip_handle(void)
{
    return &drv_epic.ezip_handle;
}
#endif

__ROM_USED EPIC_HandleTypeDef *drv_get_epic_handle(void)
{
    return &epic_handle;
}

__ROM_USED void EPIC_IRQHandler(void)
{
    EPIC_HandleTypeDef *epic;

    rt_interrupt_enter();

    epic = drv_get_epic_handle();
    RT_ASSERT(RT_NULL != epic);

    HAL_EPIC_IRQHandler(epic);

    rt_interrupt_leave();
}

#ifdef HAL_EZIP_MODULE_ENABLED
__ROM_USED void EZIP_IRQHandler(void)
{
    EZIP_HandleTypeDef *ezip;

    rt_interrupt_enter();

    ezip = drv_get_ezip_handle();
    RT_ASSERT(RT_NULL != ezip);

    HAL_EZIP_IRQHandler(ezip);

    rt_interrupt_leave();
}
#endif

static uint32_t GetElapsedUs(uint32_t prev_tick, uint32_t cur_tick)
{
    if (0 == drv_epic.hclk_freq_Mhz)
    {
        drv_epic.hclk_freq_Mhz = HAL_RCC_GetHCLKFreq(CORE_ID_CURRENT) / 1000000;
    }

    return HAL_GetElapsedTick(prev_tick, cur_tick) / drv_epic.hclk_freq_Mhz;
}

RT_WEAK uint8_t drv_gpu_is_cached_ram(uint32_t start, uint32_t len)
{
    return 1;
}

static int dcache_clean(void *data, uint32_t size)
{
    if (0 == drv_gpu_is_cached_ram((uint32_t)data, size))
        return 0;
    else
        return mpu_dcache_clean(data, size);
}

static int dcache_invalidate(void *data, uint32_t size)
{
    if (0 == drv_gpu_is_cached_ram((uint32_t)data, size))
        return 0;
    else
        return mpu_dcache_invalidate(data, size);
}

static rt_err_t epic_sem_take(rt_uint32_t ms)
{
    rt_err_t err;

    //rt_kprintf("epic_sem_take: %d\n", epic_sema.value);
    TICK_TIME_START;
    uint32_t start_ms = rt_tick_get_millisecond();

    err = rt_sem_take(&epic_sema, rt_tick_from_millisecond(ms));

    if (drv_epic.gpu_log_level & 0x20)
    {
        TICK_TIME_PRINT;
    }

#ifdef DRV_EPIC_NEW_API
    drv_epic.rd_epic_wait  += rt_tick_get_millisecond() - start_ms;
#endif /* DRV_EPIC_NEW_API */

    //rt_kprintf("epic_sem_take succ: %d\n, calladdr %p\n", epic_sema.value, RETURN_ADDR);

    return err;
}

static rt_err_t epic_sem_trytake(void)
{
    rt_err_t err;

    err = rt_sem_trytake(&epic_sema);

    return err;
}


static rt_err_t epic_sem_release(void)
{
    rt_err_t err;

    //rt_kprintf("epic_sem_release: %d\n", epic_sema.value);
    RT_ASSERT(0 == epic_sema.value);
    err = rt_sem_release(&epic_sema);
    //rt_kprintf("epic_sem_release succ: %d\n, calladdr %p\n", epic_sema.value, RETURN_ADDR);

    return err;
}
#ifdef PKG_USING_SYSTEMVIEW
#include "SEGGER_SYSVIEW.h"
#define EPIC_SYSTEMVIEW_MARK_ID 0xAAAAAAAA
static void SystemView_mark_start(const char *desc)
{
    SEGGER_SYSVIEW_OnUserStart(EPIC_SYSTEMVIEW_MARK_ID);
    if (desc) SEGGER_SYSVIEW_Print(desc);
}

static void SystemView_mark_stop(void)
{
    SEGGER_SYSVIEW_OnUserStop(EPIC_SYSTEMVIEW_MARK_ID);
}
#endif

static void (*func_debug_mark_start)(const char *) = NULL;
static void (*func_debug_mark_stop)(void) = NULL;

const char *op_name(drv_epic_op_type_t op)
{
    switch (op)
    {
    case DRV_EPIC_COLOR_BLEND:
        return "COLOR_BLEND";
    case DRV_EPIC_COLOR_FILL:
        return "FILL";
    case DRV_EPIC_IMG_ROT:
        return "IMG_ROT";
    case DRV_EPIC_IMG_COPY:
        return "COPY";
    case DRV_EPIC_LETTER_BLEND:
        return "LETTER_BLEND";
    case DRV_EPIC_TRANSFORM:
        return "TRANSFORM";


    default:
        return "UNKNOW";
    }
}

static void gpu_log_start(drv_epic_op_type_t ops,
                          void *p1,
                          void *p2,
                          void *p3)
{

#ifdef PKG_USING_SYSTEMVIEW
    static char str[64];

    switch (ops)
    {
    case DRV_EPIC_COLOR_BLEND:
    case DRV_EPIC_LETTER_BLEND:
    case DRV_EPIC_TRANSFORM:
    case DRV_EPIC_IMG_COPY:
    case DRV_EPIC_IMG_ROT:
    {
        EPIC_LayerConfigTypeDef *dst = p3;
        RT_ASSERT(NULL != dst);
        rt_int32_t len;

        len = rt_sprintf(str, "EPIC %s area=%d,%d,%d,%d", op_name(ops),
                         dst->x_offset,
                         dst->y_offset,
                         dst->x_offset + dst->width  - 1,
                         dst->y_offset + dst->height - 1);

        if (ops == DRV_EPIC_IMG_ROT)
        {

            EPIC_LayerConfigTypeDef *input_layers = p1;
            EPIC_LayerConfigTypeDef *fg = input_layers + 1;
            EPIC_TransformCfgTypeDef *rot_cfg = &(fg->transform_cfg);


            rt_sprintf(&str[len], " angle=%d, scale=%d", rot_cfg->angle, rot_cfg->scale_x);

        }
    }
    break;

    case DRV_EPIC_COLOR_FILL:
    {
        EPIC_LayerConfigTypeDef *param = p3;
        RT_ASSERT(NULL != param);


        rt_sprintf(str, "EPIC Fill opa=%d, color=0x%x w,h=%d,%d",
                   param->alpha,
                   (((uint32_t)param->color_r) << 16) | (((uint32_t)param->color_g) << 8) | (((uint32_t)param->color_b) << 0),
                   param->width, param->height
                  );
    }
    break;

    case DRV_EPIC_FILL_GRAD:
    {
        EPIC_GradCfgTypeDef *param = p3;

        rt_sprintf(str, "EPIC Fill Grad color=%08x,%08x,%08x,%08x w,h=%d,%d",
                   param->color[0][0].full, param->color[0][1].full,
                   param->color[1][0].full, param->color[1][1].full,
                   param->width, param->height);
    }
    break;


    default:
        rt_sprintf(str, "EPIC operations: %s", op_name(ops));

        break;
    }

    SystemView_mark_start(str);
#endif /* PKG_USING_SYSTEMVIEW */
}

static void gpu_log_stop(void)
{
#ifdef PKG_USING_SYSTEMVIEW
    SystemView_mark_stop();
#endif /* PKG_USING_SYSTEMVIEW */
}

static void gpu_lock(drv_epic_op_type_t ops,
                     void *p1,
                     void *p2,
                     void *p3)
{
    gpu_log_start(ops, p1, p2, p3);

    RT_ASSERT((0 == drv_epic.gpu_fg_addr) && (0 == drv_epic.gpu_bg_addr) && (0 == drv_epic.gpu_mask_addr));

    uint32_t mask_size = 0, fg_size = 0, bg_size = 0;

    /*Get fg,bg address and output area*/
    switch (ops)
    {
    case DRV_EPIC_COLOR_BLEND:
    case DRV_EPIC_LETTER_BLEND:
    {
        EPIC_LayerConfigTypeDef *fg = p1;
        EPIC_LayerConfigTypeDef *bg = p2;
        EPIC_LayerConfigTypeDef *dst = p3;
        RT_ASSERT(NULL != dst);

        EPIC_TransformCfgTypeDef *rot_cfg = &(fg->transform_cfg);

        drv_epic.gpu_fg_addr = (uint32_t) fg->data;
        drv_epic.gpu_bg_addr = (uint32_t) bg->data;

        bg_size = bg->data_size;
        fg_size = fg->data_size;

        drv_epic.gpu_output_addr = (uint32_t) dst->data;
        drv_epic.gpu_output_size = dst->data_size;

    }
    break;

    case DRV_EPIC_IMG_ROT:
    case DRV_EPIC_TRANSFORM:
    {

        EPIC_LayerConfigTypeDef *input_layers = p1;
        uint8_t input_layer_cnt = *((uint8_t *)p2);


        EPIC_LayerConfigTypeDef *bg = input_layers + 0;
        EPIC_LayerConfigTypeDef *fg = input_layers + 1;
        EPIC_LayerConfigTypeDef *mask = input_layer_cnt > 2 ? input_layers + 2 : NULL;
        EPIC_LayerConfigTypeDef *dst = p3;
        RT_ASSERT(NULL != dst);

        EPIC_TransformCfgTypeDef *rot_cfg = &(fg->transform_cfg);

        if (mask)
        {
            drv_epic.gpu_mask_addr = (uint32_t) mask->data;
            mask_size = mask->data_size;
        }
        drv_epic.gpu_fg_addr = (uint32_t) fg->data;
        drv_epic.gpu_bg_addr = (uint32_t) bg->data;

        bg_size = bg->data_size;
        fg_size = fg->data_size;

        drv_epic.gpu_output_addr = (uint32_t) dst->data;
        drv_epic.gpu_output_size = dst->data_size;

    }
    break;

    case DRV_EPIC_COLOR_FILL:
    {

        EPIC_LayerConfigTypeDef *mask = p1;

        EPIC_LayerConfigTypeDef *param = p3;
        RT_ASSERT(NULL != param);

        drv_epic.gpu_output_addr = (uint32_t) param->data;
        drv_epic.gpu_output_size = param->data_size;

        if (mask)
        {
            drv_epic.gpu_mask_addr = (uint32_t) mask->data;
            mask_size = mask->data_size;
        }


    }
    break;

    case DRV_EPIC_IMG_COPY:
    {

        EPIC_BlendingDataType *fg = p1;

        EPIC_BlendingDataType *dst = p3;

        drv_epic.gpu_fg_addr = (uint32_t) fg->data;
        drv_epic.gpu_bg_addr = drv_epic.gpu_fg_addr;
        fg_size = fg->data_size;
        bg_size = fg_size;

        drv_epic.gpu_output_addr = (uint32_t) dst->data;
        drv_epic.gpu_output_size = dst->data_size;
    }
    break;

    case DRV_EPIC_FILL_GRAD:
    {
        EPIC_GradCfgTypeDef *param = p3;

        drv_epic.gpu_output_addr = (uint32_t) param->start;
        drv_epic.gpu_output_size = RT_ALIGN((param->total_width * param->height
                                             * HAL_EPIC_GetColorDepth(param->color_mode)), 8) >> 3;
    }
    break;

    default:
        RT_ASSERT(0);
        break;
    }

    drv_epic.gpu_last_op = ops;

#ifdef PSRAM_CACHE_WB
    {
        if ((drv_epic.gpu_bg_addr != 0) && (0 == bg_size)) bg_size = UINT32_MAX;
        if ((drv_epic.gpu_fg_addr != 0) && (0 == fg_size)) fg_size = UINT32_MAX;
        if ((drv_epic.gpu_mask_addr != 0) && (0 == mask_size)) mask_size = UINT32_MAX;

        int r = 0;
        if (bg_size)
            r = dcache_clean((void *)drv_epic.gpu_bg_addr, bg_size);
        if (fg_size && (r == 0))
            r = dcache_clean((void *)drv_epic.gpu_fg_addr, fg_size);
        if (mask_size && (r == 0))
            r = dcache_clean((void *)drv_epic.gpu_mask_addr, mask_size);
        if (drv_epic.gpu_output_size && (r == 0))
            r = dcache_clean((void *)drv_epic.gpu_output_addr, drv_epic.gpu_output_size);
    }
#endif

    /* Lock GPU used flash*/
    rt_flash_lock(drv_epic.gpu_fg_addr);

    if (rt_flash_get_handle_by_addr(drv_epic.gpu_fg_addr) != rt_flash_get_handle_by_addr(drv_epic.gpu_bg_addr))
        rt_flash_lock(drv_epic.gpu_bg_addr);

    if ((rt_flash_get_handle_by_addr(drv_epic.gpu_fg_addr) != rt_flash_get_handle_by_addr(drv_epic.gpu_mask_addr))
            && (rt_flash_get_handle_by_addr(drv_epic.gpu_bg_addr) != rt_flash_get_handle_by_addr(drv_epic.gpu_mask_addr)))
        rt_flash_lock(drv_epic.gpu_mask_addr);


#ifdef BSP_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif /*BSP_USING_PM*/
    GPU_DEVICE_START();

}

static void gpu_unlock(void)
{
    GPU_DEVICE_STOP();
#ifdef BSP_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif /*BSP_USING_PM*/


    /* UnLock GPU used flash*/
    rt_flash_unlock(drv_epic.gpu_fg_addr);

    if (rt_flash_get_handle_by_addr(drv_epic.gpu_fg_addr) != rt_flash_get_handle_by_addr(drv_epic.gpu_bg_addr))
        rt_flash_unlock(drv_epic.gpu_bg_addr);

    if ((rt_flash_get_handle_by_addr(drv_epic.gpu_fg_addr) != rt_flash_get_handle_by_addr(drv_epic.gpu_mask_addr))
            && (rt_flash_get_handle_by_addr(drv_epic.gpu_bg_addr) != rt_flash_get_handle_by_addr(drv_epic.gpu_mask_addr)))
        rt_flash_unlock(drv_epic.gpu_mask_addr);


    dcache_invalidate((void *)drv_epic.gpu_output_addr, drv_epic.gpu_output_size);


    drv_epic.gpu_fg_addr  = 0;
    drv_epic.gpu_bg_addr  = 0;
    drv_epic.gpu_mask_addr = 0;

    drv_epic.gpu_output_addr = 0;
    drv_epic.gpu_output_size = 0;

    gpu_log_stop();
}



rt_err_t drv_gpu_take(rt_int32_t ms)
{
    rt_err_t err;

    err = epic_sem_take(ms);

    if (RT_EOK != err)
    {
        print_gpu_error_info();
    }

    return err;
}

rt_err_t drv_gpu_release(void)
{
    return epic_sem_release();
}


rt_err_t drv_gpu_check_done(rt_int32_t ms)
{
    rt_err_t err;

    if (epic_sema.value == 0) // Speed up return result if GPU is NOT working
    {
        err = epic_sem_take(ms);
        if (RT_EOK != err)
        {
            print_gpu_error_info();
        }
        epic_sem_release();
    }
    else
    {
        err = RT_EOK;
    }


    return err;
}



void drv_gpu_open(void)
{
#ifdef HAL_EPIC_MODULE_ENABLED
    if (NULL == epic)
    {
        RT_ASSERT(1 == drv_epic_inited);
        epic = drv_get_epic_handle();

        HAL_NVIC_SetPriority(EPIC_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(EPIC_IRQn);

        epic->Instance = EPIC;
        epic->RamInstance = &drv_epic.RamEPIC;

#ifdef EPIC_DEBUG
        epic->op_hist = &drv_epic.epic_op_hist;
#endif /* EPIC_DEBUG */

#ifdef HAL_EZIP_MODULE_ENABLED
        HAL_NVIC_SetPriority(EZIP_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(EZIP_IRQn);

        epic->hezip = drv_get_ezip_handle();
        epic->hezip->Instance = EZIP;
#if ((defined(BSP_QSPI3_MODE) && (BSP_QSPI3_MODE == 1)) || (defined(BSP_QSPI2_MODE) && (BSP_QSPI2_MODE == 1)) || (defined(BSP_QSPI4_MODE) && (BSP_QSPI4_MODE == 1)))
        epic->hezip->flash_handle_query_cb = (EZIP_FlashHandleQueryCbTypeDef)rt_flash_get_handle_by_addr;
#elif defined(SF32LB56X)
        epic->hezip->flash_handle_query_cb = (EZIP_FlashHandleQueryCbTypeDef)rt_flash_get_handle_by_addr;
#else
        epic->hezip->flash_handle_query_cb = NULL;
#endif
        epic->hezip->RamInstance = &drv_epic.RamEZIP;

        HAL_EZIP_Init(epic->hezip);
#endif
        HAL_EPIC_Init(epic);


#ifdef DRV_EPIC_NEW_API
        memcpy(&drv_epic.epic_handle2, &epic_handle, sizeof(drv_epic.epic_handle2));
#ifdef HAL_EZIP_MODULE_ENABLED
        memcpy(&drv_epic.ezip_handle2, &drv_epic.ezip_handle, sizeof(drv_epic.ezip_handle2));
#endif
        drv_epic.epic_handle2.hezip = &drv_epic.ezip_handle2;
#endif /* DRV_EPIC_NEW_API */
        LOG_I("drv_gpu opened.");
    }
#endif
}


void drv_gpu_close(void)
{
#ifdef HAL_EPIC_MODULE_ENABLED
    if (epic)
    {
        RT_ASSERT(1 == drv_epic_inited);
        drv_gpu_check_done(GPU_BLEND_EXP_MS);
        RT_ASSERT(epic->State != HAL_EPIC_STATE_BUSY);
#ifdef HAL_EZIP_MODULE_ENABLED
        if (epic->hezip)
        {
            RT_ASSERT(epic->hezip->State != HAL_EZIP_STATE_BUSY);
            HAL_EZIP_DeInit(epic->hezip);
        }
#endif /* HAL_EZIP_MODULE_ENABLED */

        epic = NULL;
        LOG_I("drv_gpu closed.");
    }
#endif
}


static bool is_repeated_mem(uint32_t *start, uint32_t *end)
{
    uint32_t repeat_v = *start;

    for (uint32_t *ptr = start; ptr < end; ptr++)
    {
        if (repeat_v != (*ptr)) return false;
    }

    return true;
}
#ifdef HAL_EPIC_MODULE_ENABLED
#ifdef EPIC_DEBUG
static void print_LayerInfo(const char *name, const EPIC_BlendingDataType *p_layer)
{
    LOG_E("%s cf=0x%x,data=0x%x,total_w=%d,area[x0y0(%d,%d),x1y1(%d,%d)] frac[%x,%x]",
          name, p_layer->color_mode, p_layer->data, p_layer->total_width,
          p_layer->x_offset, p_layer->y_offset,
          p_layer->x_offset + p_layer->width - 1,
          p_layer->y_offset + p_layer->height - 1,
          p_layer->x_offset_frac, p_layer->y_offset_frac
         );

    LOG_E("color_en=%d, rgb[%x,%x,%x], ax=%d, L8_tab=0x%x, data_size=0x%x",
          p_layer->color_en, p_layer->color_r, p_layer->color_g, p_layer->color_b,
          p_layer->ax_mode, p_layer->lookup_table, p_layer->data_size);
}

static void print_TransInfo(const EPIC_TransformCfgTypeDef *p_tcfg)
{
    LOG_E("angle=%d,scale_xy=%d,%d, pivot_xy=%d,%d, mirror_hv=%d,%d",
          p_tcfg->angle, p_tcfg->scale_x, p_tcfg->scale_y,
          p_tcfg->pivot_x, p_tcfg->pivot_y,
          p_tcfg->h_mirror, p_tcfg->v_mirror);

}
static void print_FillInfo(const EPIC_FillingCfgTypeDef *p_fill)
{
    LOG_E("cf=0x%x,data=0x%x,total_w=%d,wh=%d,%d,argb[%x,%x,%x,%x]",
          p_fill->color_mode, p_fill->start, p_fill->total_width,
          p_fill->width, p_fill->height,
          p_fill->alpha, p_fill->color_r, p_fill->color_g, p_fill->color_b);
}
static void print_GradInfo(const EPIC_GradCfgTypeDef *p_grad)
{
    LOG_E("cf=0x%x,data=0x%x,total_w=%d,wh=%d,%d",
          p_grad->color_mode, p_grad->start, p_grad->total_width,
          p_grad->width, p_grad->height);
}

static void print_ExLayerInfo(const char *name, const EPIC_LayerConfigTypeDef *p_layer_ex)
{
    print_LayerInfo(name, (const EPIC_BlendingDataType *)p_layer_ex);
    print_TransInfo(&p_layer_ex->transform_cfg);
    LOG_E("alpha=%d \r\n", p_layer_ex->alpha);
}

static void print_epic_op_history(void)
{
    LOG_E("epic_op_history cur=%d", epic->op_hist->idx);
    uint8_t i = epic->op_hist->idx;

    {

        LOG_E("hist[%d]", i);
        EPIC_OpParamTypeDef *param = &(epic->op_hist->hist[i].param);
        switch (epic->op_hist->hist[i].op)
        {
        case EPIC_OP_ROTATION:
        {
            LOG_E("EPIC_OP_ROTATION   alpha=%d", param->rot_param.alpha);

            print_LayerInfo("fg", &param->rot_param.fg);
            print_LayerInfo("bg", &param->rot_param.bg);
            print_LayerInfo("dst", &param->rot_param.dst);
            print_TransInfo(&param->rot_param.rot_cfg);
        }
        break;

        case EPIC_OP_BLENDING:
        {
            LOG_E("EPIC_OP_BLENDING   alpha=%d", param->blend_param.alpha);

            print_LayerInfo("fg", &param->blend_param.fg);
            print_LayerInfo("bg", &param->blend_param.bg);
            print_LayerInfo("dst", &param->blend_param.dst);
        }
        break;

        case EPIC_OP_FILLING:
        {
            LOG_E("EPIC_OP_FILLING");

            print_FillInfo(&param->fill_param);
        }
        break;

        case EPIC_OP_FillGrad:
        {
            LOG_E("EPIC_OP_FillGrad");

            print_GradInfo(&param->grad_param);
        }
        break;

        case EPIC_OP_BLENDING_EX:
        {
            LOG_E("EPIC_OP_BLENDING_EX   layer_num=%d", param->blend_ex_param.input_layer_num);

            for (uint8_t j = 0; j < param->blend_ex_param.input_layer_num; j++)
            {
                char name[3] = {'#', '0', '\0'};
                name[1] = '0' + j;
                print_ExLayerInfo(name, &param->blend_ex_param.input_layer[j]);
            }
            print_LayerInfo("output", (const EPIC_BlendingDataType *)&param->blend_ex_param.output_layer);
        }
        break;

        case EPIC_OP_COPY:
        {
            LOG_E("EPIC_OP_COPY");

            print_LayerInfo("src", &param->copy_param.src);
            print_LayerInfo("dst", &param->copy_param.dst);
        }
        break;

        case EPIC_OP_TRANSFORM:
        {
            LOG_E("EPIC_OP_TRANSFORM   layer_num=%d", param->transform_param.input_layer_num);
            LOG_E("hor_path=%p ver_path=%p user_data=%p", param->transform_param.hor_path,
                  param->transform_param.ver_path,
                  param->transform_param.user_data);

            for (uint8_t j = 0; j < param->transform_param.input_layer_num; j++)
            {
                char name[3] = {'#', '0', '\0'};
                name[1] = '0' + j;
                print_ExLayerInfo(name, &param->transform_param.input_layer[j]);
            }
            print_LayerInfo("output", (const EPIC_BlendingDataType *)&param->transform_param.output_layer);
        }
        break;

        default:
            break;
        }
    }
}
#endif /* EPIC_DEBUG */
#endif /* HAL_EPIC_MODULE_ENABLED */

static void gpu_reset(void)
{
#ifdef HAL_EPIC_MODULE_ENABLED
    if (epic)
    {
        rt_base_t level;
        level = rt_hw_interrupt_disable();

        epic->State = HAL_EPIC_STATE_READY;
        epic->ErrorCode = 0;
        epic->IntXferCpltCallback = NULL;
        HAL_RCC_ResetModule(RCC_MOD_EPIC);
#ifdef HAL_EZIP_MODULE_ENABLED
        epic->hezip->State = HAL_EZIP_STATE_READY;
        epic->hezip->ErrorCode = 0;
        epic->hezip->CpltCallback = NULL;
        HAL_RCC_ResetModule(RCC_MOD_EZIP);
        HAL_NVIC_ClearPendingIRQ(EZIP_IRQn);
#endif /* HAL_EZIP_MODULE_ENABLED */
        HAL_NVIC_ClearPendingIRQ(EPIC_IRQn);

        rt_hw_interrupt_enable(level);
    }
#endif /* HAL_EPIC_MODULE_ENABLED */
}


static void print_gpu_error_info(void)
{
    LOG_E("print_gpu_error_info");
    drv_epic.gpu_timeout_cnt++;

#ifdef HAL_EPIC_MODULE_ENABLED
    RT_ASSERT(epic != NULL);

    if ((HAL_EPIC_STATE_READY != epic->State) || (epic->Instance->STATUS & EPIC_STATUS_IA_BUSY))
    {
        LOG_E("Epic not ready %d, HW busy =%d, ErrorCode %xh", epic->State, (epic->Instance->STATUS & EPIC_STATUS_IA_BUSY),
              epic->ErrorCode);

        if (is_repeated_mem((uint32_t *) epic->Instance, (uint32_t *)(epic->Instance + 1)))
        {
            LOG_E("Epic not open");
        }
        else
        {
            LOG_E("EOF IRQ=%x, MASK=%x", epic->Instance->EOF_IRQ & (EPIC_EOF_IRQ_IRQ_STATUS_Msk | EPIC_EOF_IRQ_IRQ_CAUSE_Msk),
                  epic->Instance->SETTING & EPIC_SETTING_EOF_IRQ_MASK);


            if (epic->Instance->VL_CFG & EPIC_VL_CFG_ACTIVE)
            {
                LOG_E("VL SRC %x, x0y0x1y1[%d,%d,%d,%d]", epic->Instance->VL_SRC,
                      epic->Instance->VL_TL_POS & 0xFFFF, epic->Instance->VL_TL_POS >> 16,
                      epic->Instance->VL_BR_POS & 0xFFFF, epic->Instance->VL_BR_POS >> 16);

#if defined(SF32LB55X) || defined(SF32LB58X)
                if (epic->Instance->VL_CFG & EPIC_VL_CFG_EZIP_EN)
                {
                    LOG_E("VL is Ezip");
                }
#endif /* SF32LB55X || SF32LB58X */

            }

            EPIC_LayerxTypeDef *layer_x;
            layer_x = (EPIC_LayerxTypeDef *)&epic->Instance->L0_CFG;
            for (uint32_t layer_idx = 0; layer_idx < 2; layer_idx++)
            {
                layer_x += layer_idx;
                if (layer_x->CFG & EPIC_L0_CFG_ACTIVE)
                {
                    LOG_E("L%d SRC %x, x0y0x1y1[%d,%d,%d,%d]", layer_idx, layer_x->SRC,
                          layer_x->TL_POS & 0xFFFF, layer_x->TL_POS >> 16,
                          layer_x->BR_POS & 0xFFFF, layer_x->BR_POS >> 16);

#if defined(SF32LB55X) || defined(SF32LB58X)
                    if (layer_x->CFG & EPIC_L0_CFG_EZIP_EN)
                    {
                        LOG_E("L%d is Ezip", layer_idx);
                    }
#endif /* SF32LB55X || SF32LB58X */
                }
            }


            layer_x = (EPIC_LayerxTypeDef *)&epic->Instance->L2_CFG;
            if (layer_x->CFG & EPIC_L2_CFG_ACTIVE)
            {
                LOG_E("L2 SRC %x, x0y0x1y1[%d,%d,%d,%d]", layer_x->SRC,
                      layer_x->TL_POS & 0xFFFF, layer_x->TL_POS >> 16,
                      layer_x->BR_POS & 0xFFFF, layer_x->BR_POS >> 16);

#if defined(SF32LB55X) || defined(SF32LB58X)
                if (layer_x->CFG & EPIC_L2_CFG_EZIP_EN)
                {
                    LOG_E("L2 is Ezip");
                }
#endif /* SF32LB55X || SF32LB58X */
            }



#if !(defined(SF32LB55X)||defined(SF32LB58X))
            if (epic->Instance->COENG_CFG & EPIC_COENG_CFG_EZIP_EN)
            {
                LOG_E("%d is Ezip(0:VL 1:L0 2:L1 3:L2)", (epic->Instance->COENG_CFG & EPIC_COENG_CFG_EZIP_CH_SEL_Msk) >> EPIC_COENG_CFG_EZIP_CH_SEL_Pos);
            }
#endif
        }


    }
    else
    {
        LOG_E("Epic is ready");
    }




#ifdef HAL_EZIP_MODULE_ENABLED
    EZIP_HandleTypeDef        *hezip = epic->hezip;

    RT_ASSERT(hezip != NULL);
    if ((HAL_EZIP_STATE_READY != hezip->State) || (hezip->Instance->EZIP_CTRL & EZIP_EZIP_CTRL_EZIP_CTRL))
    {
        LOG_E("Ezip not ready %d, HW busy=%x, ErrorCode %xh", hezip->State, (hezip->Instance->EZIP_CTRL & EZIP_EZIP_CTRL_EZIP_CTRL),
              hezip->ErrorCode);
        if (is_repeated_mem((uint32_t *) hezip->Instance, (uint32_t *)(hezip->Instance + 1)))
        {
            LOG_E("Ezip not open");
        }
        else
        {

            LOG_E("Ezip src=%x, area x0y0x1y1[%d,%d,%d,%d]", hezip->Instance->SRC_ADDR,
                  hezip->Instance->START_POINT >> 16, hezip->Instance->START_POINT & 0xFFFF,
                  hezip->Instance->END_POINT >> 16, hezip->Instance->END_POINT & 0xFFFF);

            LOG_E("type %d (1~4:ezip, 5:aezip) width:%d, height:%d",
                  hezip->Instance->DB_DATA3 >> 24,
                  hezip->Instance->DB_DATA1 >> 16,
                  0xFFFF & (hezip->Instance->DB_DATA1));

            LOG_E("data size=%x", hezip->Instance->DB_DATA2);
            LOG_E("ezip_buf_full=%d", 1 & (hezip->Instance->DB_DATA4 >> 9)); //if 1, wait epic read data
            LOG_E("Ezip cur x:%d, y:%d", (hezip->Instance->DB_DATA5 >> 16) - 1, (0xFFFF & (hezip->Instance->DB_DATA5)) - 1) ;
        }
    }
    else
    {
        LOG_E("Ezip is ready");
    }

#endif /* HAL_EZIP_MODULE_ENABLED */

#ifdef EPIC_DEBUG
    print_epic_op_history();
#endif /* EPIC_DEBUG */


#endif /* HAL_EPIC_MODULE_ENABLED */
}


static uint8_t *get_map_ptr_by_xy(const uint8_t *map, uint8_t pixel_depth, uint32_t total_width, int32_t x, int32_t y)
{
    uint32_t offset_pixels = total_width * y + x;
    if (0 == (pixel_depth & 0x7))
    {
        return (uint8_t *)(map + (pixel_depth >> 3) * offset_pixels);
    }
    else
    {
        uint32_t offset_bits = offset_pixels * pixel_depth;
        RT_ASSERT(0 == (offset_bits & 0x7));

        return (uint8_t *)(map + (offset_bits >> 3));
    }
}

static uint32_t get_layer_size(EPIC_BlendingDataType *p_layer)
{
    uint32_t pixel_size = HAL_EPIC_GetColorDepth(p_layer->color_mode);

    return ((pixel_size * p_layer->total_width * p_layer->height) + 7) >> 3;
}

static void clip_layer_to_area(EPIC_BlendingDataType *p_layer,
                               const uint8_t *data,
                               int16_t x, int16_t y,
                               const EPIC_AreaTypeDef *copy_area)
{
    uint32_t pixel_size = HAL_EPIC_GetColorDepth(p_layer->color_mode);

    RT_ASSERT(!EPIC_IS_EZIP_COLOR_MODE(p_layer->color_mode));
    RT_ASSERT(!EPIC_IS_YUV_COLOR_MODE(p_layer->color_mode));


    //Clip dst layer to copy area
    p_layer->width = HAL_EPIC_AreaWidth(copy_area);
    p_layer->height = HAL_EPIC_AreaHeight(copy_area);
    p_layer->data = get_map_ptr_by_xy(data, pixel_size, p_layer->total_width,
                                      copy_area->x0 - x, copy_area->y0 - y);
    p_layer->x_offset = copy_area->x0;
    p_layer->y_offset = copy_area->y0;


    p_layer->data_size = ((pixel_size * p_layer->total_width * p_layer->height) + 7) >> 3;
}






#ifndef DRV_EPIC_NEW_API
static HAL_StatusTypeDef epic_split_render_next(uint8_t init);

static void epic_cplt_callback(EPIC_HandleTypeDef *epic)
{
    drv_epic_cplt_cbk cb;

    if (DRV_EPIC_INVALID != drv_epic.split_rd.op)
    {
        HAL_StatusTypeDef hal_err = epic_split_render_next(0);
        if (HAL_OK == hal_err)
        {
            return;
        }
        else
        {
            drv_epic.split_rd.op = DRV_EPIC_INVALID;
            LOG_E("epic_split_render ABORTED, err=%d", hal_err);
        }
    }

    cb = drv_epic.cbk;
    drv_epic.cbk = NULL;

    rt_err_t err;
    gpu_unlock();

    err = epic_sem_release();
    RT_ASSERT(RT_EOK == err);



    if (cb) cb(epic);
}

static void epic_abort_callback(EPIC_HandleTypeDef *epic)
{
    drv_epic.split_rd.op = DRV_EPIC_INVALID;
    epic_cplt_callback(epic);
}


static rt_err_t wait_gpu_done(rt_int32_t time)
{
    rt_err_t err;

    err = drv_gpu_take(time);

    if (RT_EOK != err)
    {
        LOG_E("wait_gpu_done timeout(-2)? err=%d", err);
#ifdef __RELEASE__
        gpu_reset();
        epic_abort_callback(&epic_handle);
#else
        RT_ASSERT(0); //Raise an assertion in debug mode.
#endif /* __RELEASE__ */
    }

    return err;
}

static HAL_StatusTypeDef epic_split_render_next(uint8_t init)
{
    EPIC_AreaTypeDef *p_render_area = &drv_epic.split_rd.render_area;
    EPIC_AreaTypeDef *p_next_sub_area = &drv_epic.split_rd.next_render_area;
    drv_epic_op_type_t  cur_op = drv_epic.split_rd.op;
    EPIC_AreaTypeDef cur_render_area;
    HAL_StatusTypeDef ret;

    //Get current rendering area
    if (init)
    {
        cur_render_area.x0 = p_render_area->x0;
        cur_render_area.x1 = MIN(p_render_area->x0 + EPIC_COORDINATES_MAX - 1, p_render_area->x1);

        cur_render_area.y0 = p_render_area->y0;
        cur_render_area.y1 = MIN(p_render_area->y0 + EPIC_COORDINATES_MAX - 1, p_render_area->y1);

        drv_epic.split_rd.next_render_area = cur_render_area;
    }
    else
    {
        cur_render_area = drv_epic.split_rd.next_render_area;
    }

    //Move p_next_sub_area' to next area
    if (p_next_sub_area->x0 + EPIC_COORDINATES_MAX <= p_render_area->x1)
    {
        p_next_sub_area->x0 = p_next_sub_area->x0 + EPIC_COORDINATES_MAX;
        p_next_sub_area->x1 = MIN(p_next_sub_area->x0 + EPIC_COORDINATES_MAX - 1, p_render_area->x1);
    }
    else if (p_next_sub_area->y0 + EPIC_COORDINATES_MAX <= p_render_area->y1)
    {
        p_next_sub_area->x0 = p_render_area->x0;
        p_next_sub_area->x1 = MIN(p_next_sub_area->x0 + EPIC_COORDINATES_MAX - 1, p_render_area->x1);

        p_next_sub_area->y0 = p_next_sub_area->y0 + EPIC_COORDINATES_MAX;
        p_next_sub_area->y1 = MIN(p_next_sub_area->y0 + EPIC_COORDINATES_MAX - 1, p_render_area->y1);
    }
    else
    {
        drv_epic.split_rd.op = DRV_EPIC_INVALID; //Render_area done.
    }


    //Clip dst layer to current rendering area
    EPIC_BlendingDataType *p_dst_layer = (EPIC_BlendingDataType *) &drv_epic.output_layer;
    const EPIC_AreaTypeDef *dst_area = &drv_epic.split_rd.dst_area;
    clip_layer_to_area(p_dst_layer, drv_epic.split_rd.dst_data,
                       dst_area->x0, dst_area->y0, &cur_render_area);


    //Start it.
    LOG_D("split_render [%s] "AreaString" data=0x%x", op_name(cur_op), AreaParams(&cur_render_area),
          p_dst_layer->data);
    EPIC_HandleTypeDef *h_epic = &epic_handle;
    switch (cur_op)
    {
    case DRV_EPIC_IMG_COPY:
    {
        EPIC_BlendingDataType *p_src_layer = (EPIC_BlendingDataType *) &drv_epic.input_layers[0];


        PRINT_LAYER_INFO(p_src_layer, "src");
        PRINT_LAYER_INFO(p_dst_layer, "dst");
        h_epic->XferCpltCallback = epic_cplt_callback;
        ret = HAL_EPIC_Copy_IT(h_epic, p_src_layer, p_dst_layer);
    }
    break;

    case DRV_EPIC_COLOR_FILL:
    {
        LOG_D("fill color %02x%02x%02x", drv_epic.output_layer.color_r,
              drv_epic.output_layer.color_g,
              drv_epic.output_layer.color_b);
        for (uint8_t i = 0; i < drv_epic.input_layer_cnt; i++)
        {
            PRINT_LAYER_INFO(&drv_epic.input_layers[i], "src");
        }
        PRINT_LAYER_INFO(&drv_epic.output_layer, "dst");
        h_epic->XferCpltCallback = epic_cplt_callback;
        ret = HAL_EPIC_BlendStartEx_IT(h_epic, drv_epic.input_layers,
                                       drv_epic.input_layer_cnt, &drv_epic.output_layer);
    }
    break;

    case DRV_EPIC_IMG_ROT:
    {
        for (uint8_t i = 0; i < drv_epic.input_layer_cnt; i++)
        {
            PRINT_LAYER_INFO(&drv_epic.input_layers[i], "src");
            PRINT_LAYER_EXTRA_INFO(&drv_epic.input_layers[i]);
        }
        PRINT_LAYER_INFO(&drv_epic.output_layer, "dst");
        h_epic->XferCpltCallback = epic_cplt_callback;
        ret = HAL_EPIC_BlendStartEx_IT(h_epic, drv_epic.input_layers,
                                       drv_epic.input_layer_cnt, &drv_epic.output_layer);
    }
    break;

    case DRV_EPIC_TRANSFORM:
    {
        for (uint8_t i = 0; i < drv_epic.input_layer_cnt; i++)
        {
            PRINT_LAYER_INFO(&drv_epic.input_layers[i], "src");
            PRINT_LAYER_EXTRA_INFO(&drv_epic.input_layers[i]);
        }
        PRINT_LAYER_INFO(&drv_epic.output_layer, "dst");

        ret = HAL_EPIC_TransStart(h_epic, drv_epic.input_layers,
                                  drv_epic.input_layer_cnt, &drv_epic.output_layer,
                                  drv_epic.split_rd.hor_path,
                                  drv_epic.split_rd.ver_path,
                                  drv_epic.split_rd.user_data);

    }
    break;

    case DRV_EPIC_FILL_GRAD:
    {
        EPIC_GradCfgTypeDef param;

        param.start = p_dst_layer->data;
        param.color_mode = p_dst_layer->color_mode;
        param.width = p_dst_layer->width;
        param.height = p_dst_layer->height;
        param.total_width = p_dst_layer->total_width;
        memcpy(param.color, drv_epic.grad_color, sizeof(param.color));
        LOG_D("EPIC Fill Grad buf=0x%x, cf=%d, total_w=%d w,h=%d,%d",
              param.start, param.color_mode, param.total_width,
              param.width, param.height);


        ret = HAL_EPIC_FillGrad_IT(h_epic, &param);

    }
    break;

    default:
        RT_ASSERT(0);
        break;
    }

    return ret;
}

static void cont_blend_reset(void)
{
    if (drv_epic.cont_mode)
    {
        HAL_StatusTypeDef ret;
        EPIC_HandleTypeDef *h_epic = &epic_handle;

        LOG_D("Reset drv_epic.cont_mode");

        ret = HAL_EPIC_ContBlendStop(h_epic);

        if (HAL_OK == ret)
        {
            gpu_unlock();
            drv_epic_release();
        }
        drv_epic.cont_mode = false;
    }
}

rt_err_t drv_epic_copy(const uint8_t *src, uint8_t *dst,
                       const EPIC_AreaTypeDef *src_area,
                       const EPIC_AreaTypeDef *dst_area,
                       const EPIC_AreaTypeDef *copy_area,
                       uint32_t src_cf, uint32_t dst_cf,
                       drv_epic_cplt_cbk cbk)
{
    RT_ASSERT((RT_NULL != src_area) && (RT_NULL != dst_area) && (RT_NULL != copy_area));
    RT_ASSERT(HAL_EPIC_AreaIsIn(copy_area, src_area) && HAL_EPIC_AreaIsIn(copy_area, dst_area));

    HAL_StatusTypeDef ret;
    rt_err_t err;
    EPIC_HandleTypeDef *h_epic = &epic_handle;
    EPIC_BlendingDataType *p_src_layer = (EPIC_BlendingDataType *) &drv_epic.input_layers[0];
    EPIC_BlendingDataType *p_dst_layer = (EPIC_BlendingDataType *) &drv_epic.output_layer;

    err = wait_gpu_done(GPU_BLEND_EXP_MS);
    if (RT_EOK != err) return err;

    LOG_D("\r\ndrv_epic_copy src=%p "AreaString" to dst=%p "AreaString,
          src, AreaParams(src_area),
          dst, AreaParams(dst_area));

    LOG_D("src_cf=%d, dst_cf=%d, clip_area:"AreaString,
          src_cf, dst_cf, AreaParams(copy_area));



    HAL_EPIC_BlendDataInit(p_src_layer);
    p_src_layer->color_mode = src_cf;
    p_src_layer->total_width = HAL_EPIC_AreaWidth(src_area);
    p_src_layer->data = (uint8_t *)src;
    p_src_layer->width = HAL_EPIC_AreaWidth(src_area);
    p_src_layer->height = HAL_EPIC_AreaHeight(src_area);
    p_src_layer->x_offset = src_area->x0;
    p_src_layer->y_offset = src_area->y0;
    p_src_layer->data_size = get_layer_size(p_src_layer);

    HAL_EPIC_BlendDataInit(p_dst_layer);
    p_dst_layer->color_mode = dst_cf;
    p_dst_layer->total_width = HAL_EPIC_AreaWidth(dst_area);

    //Clip dst layer to copy area
    clip_layer_to_area(p_dst_layer, dst, dst_area->x0, dst_area->y0, copy_area);

    cont_blend_reset();
    gpu_lock(DRV_EPIC_IMG_COPY, p_src_layer, NULL, p_dst_layer);
    drv_epic.cbk = cbk;


    if ((HAL_EPIC_AreaWidth(copy_area) > EPIC_COORDINATES_MAX) || HAL_EPIC_AreaHeight(copy_area) > EPIC_COORDINATES_MAX)
    {
        memcpy(&drv_epic.split_rd.dst_area, dst_area, sizeof(EPIC_AreaTypeDef));
        drv_epic.split_rd.dst_data = dst;

        memcpy(&drv_epic.split_rd.render_area, copy_area, sizeof(EPIC_AreaTypeDef));
        drv_epic.split_rd.op = DRV_EPIC_IMG_COPY;


        ret = epic_split_render_next(1);
    }
    else
    {

        PRINT_LAYER_INFO(p_src_layer, "src");
        PRINT_LAYER_INFO(p_dst_layer, "dst");
        h_epic->XferCpltCallback = epic_cplt_callback;
        ret = HAL_EPIC_Copy_IT(h_epic, p_src_layer, p_dst_layer);
    }


    return (HAL_OK == ret) ? RT_EOK : RT_ERROR;
}

rt_err_t drv_epic_fill_ext(EPIC_LayerConfigTypeDef *input_layers,
                           uint8_t input_layer_cnt,
                           EPIC_LayerConfigTypeDef *output_canvas,
                           drv_epic_cplt_cbk cbk)
{

    RT_ASSERT(RT_NULL != output_canvas);

    HAL_StatusTypeDef ret;
    rt_err_t err;
    EPIC_HandleTypeDef *h_epic = &epic_handle;
    EPIC_AreaTypeDef *fill_area = &drv_epic.split_rd.dst_area;

    err = wait_gpu_done(GPU_BLEND_EXP_MS);
    if (RT_EOK != err) return err;

    cont_blend_reset();
    gpu_lock(DRV_EPIC_COLOR_FILL,
             (3 == input_layer_cnt) ? input_layers + 2 : NULL,
             NULL, output_canvas);
    drv_epic.cbk = cbk;

    fill_area->x0 = output_canvas->x_offset;
    fill_area->y0 = output_canvas->y_offset;
    fill_area->x1 = output_canvas->x_offset + output_canvas->width  - 1;
    fill_area->y1 = output_canvas->y_offset + output_canvas->height - 1;

    if ((HAL_EPIC_AreaWidth(fill_area) > EPIC_COORDINATES_MAX)
            || HAL_EPIC_AreaHeight(fill_area) > EPIC_COORDINATES_MAX)
    {
        if ((input_layer_cnt > 0) && (&drv_epic.input_layers[0] != input_layers))
            memcpy(&drv_epic.input_layers[0], input_layers, input_layer_cnt * sizeof(drv_epic.input_layers[0]));
        if (&drv_epic.output_layer != output_canvas)
            memcpy(&drv_epic.output_layer, output_canvas, sizeof(drv_epic.output_layer));

        drv_epic.input_layer_cnt = input_layer_cnt;
        drv_epic.split_rd.dst_data = output_canvas->data;
        memcpy(&drv_epic.split_rd.render_area, fill_area, sizeof(EPIC_AreaTypeDef));
        drv_epic.split_rd.op = DRV_EPIC_COLOR_FILL;


        ret = epic_split_render_next(1);
    }
    else
    {
        LOG_D("fill color %02x%02x%02x", output_canvas->color_r,
              output_canvas->color_g,
              output_canvas->color_b);
        for (uint8_t i = 0; i < input_layer_cnt; i++)
        {
            PRINT_LAYER_INFO(input_layers + i, "src");
        }
        PRINT_LAYER_INFO(output_canvas, "dst");
        h_epic->XferCpltCallback = epic_cplt_callback;
        ret = HAL_EPIC_BlendStartEx_IT(h_epic, input_layers, input_layer_cnt, output_canvas);
    }


    return (HAL_OK == ret) ? RT_EOK : RT_ERROR;

}

rt_err_t drv_epic_fill(uint32_t dst_cf, uint8_t *dst,
                       const EPIC_AreaTypeDef *dst_area,
                       const EPIC_AreaTypeDef *fill_area,
                       uint32_t fill_color,
                       uint32_t mask_cf, const uint8_t *mask_map,
                       const EPIC_AreaTypeDef *mask_area,
                       drv_epic_cplt_cbk cbk)
{
    RT_ASSERT(RT_NULL != dst);
    RT_ASSERT(RT_NULL != fill_area);
    RT_ASSERT(RT_NULL != dst_area);

    rt_err_t err;
    err = drv_gpu_check_done(GPU_BLEND_EXP_MS);
    if (RT_EOK != err) return err;


    LOG_D("\r\n drv_epic_fill dst=%p "AreaString" color=%x", dst, AreaParams(dst_area), fill_color);
    if (mask_map)
    {
        LOG_D("mask=%p "AreaString, mask_map, AreaParams(mask_area));
    }

    EPIC_LayerConfigTypeDef *p_output_canvas = &drv_epic.output_layer;
    uint8_t opa;


    HAL_EPIC_LayerConfigInit(p_output_canvas);
    p_output_canvas->color_mode = dst_cf;
    p_output_canvas->total_width = HAL_EPIC_AreaWidth(dst_area);
    opa = (uint8_t)((fill_color >> 24) & 0xFF);
    p_output_canvas->color_r = (uint8_t)((fill_color >> 16) & 0xFF);
    p_output_canvas->color_g = (uint8_t)((fill_color >> 8) & 0xFF);
    p_output_canvas->color_b = (uint8_t)((fill_color >> 0) & 0xFF);
    p_output_canvas->color_en = true;

    //Clip dst layer to filling area
    clip_layer_to_area((EPIC_BlendingDataType *)p_output_canvas, dst, dst_area->x0, dst_area->y0, fill_area);

    if (mask_map)
    {
#if defined(EPIC_SUPPORT_MONOCHROME_LAYER)&&defined(EPIC_SUPPORT_MASK)
        EPIC_LayerConfigTypeDef *p_input_layers = &drv_epic.input_layers[0];

        HAL_EPIC_LayerConfigInit(&p_input_layers[2]);
        p_input_layers[2].data = (uint8_t *)mask_map;
        p_input_layers[2].x_offset = mask_area->x0;
        p_input_layers[2].y_offset = mask_area->y0;
        p_input_layers[2].width = HAL_EPIC_AreaWidth(mask_area);
        p_input_layers[2].total_width = p_input_layers[2].width;
        p_input_layers[2].height = HAL_EPIC_AreaHeight(mask_area);
        p_input_layers[2].color_mode = mask_cf;
        p_input_layers[2].ax_mode = ALPHA_BLEND_MASK;
        p_input_layers[2].data_size = get_layer_size((EPIC_BlendingDataType *)&p_input_layers[2]);


        p_input_layers[1] = *p_output_canvas;
        p_input_layers[1].data = (uint8_t *) p_input_layers;//Any accessable address for mono layer
        p_input_layers[1].color_mode = EPIC_INPUT_MONO;
        p_input_layers[1].alpha = opa;
        p_input_layers[1].ax_mode = ALPHA_BLEND_RGBCOLOR;


        p_input_layers[0] = *p_output_canvas;
        p_input_layers[0].color_en = false;

        p_output_canvas->color_en = false;

        err = drv_epic_fill_ext(p_input_layers, 3, p_output_canvas, cbk);
#else
        RT_ASSERT(0);
#endif /* EPIC_SUPPORT_MONOCHROME_LAYER&&EPIC_SUPPORT_MASK */

    }
    else if (opa != 255)
    {
        EPIC_LayerConfigTypeDef *p_input_layer = &drv_epic.input_layers[0];

        p_input_layer->alpha = (0 == opa) ? 255 : (256 - opa);
        p_input_layer->color_en = false;

        err = drv_epic_fill_ext(p_input_layer, 1, p_output_canvas, cbk);
    }
    else
    {
        err = drv_epic_fill_ext(NULL, 0, p_output_canvas, cbk);
    }

    return err;
}

rt_err_t drv_epic_fill_grad(EPIC_GradCfgTypeDef *param,
                            drv_epic_cplt_cbk cbk)
{

    RT_ASSERT(RT_NULL != param);
    RT_ASSERT(RT_NULL != param->start);

    HAL_StatusTypeDef ret;
    rt_err_t err;
    EPIC_HandleTypeDef *h_epic = &epic_handle;
    EPIC_AreaTypeDef *fill_area = &drv_epic.split_rd.dst_area;

    err = wait_gpu_done(GPU_BLEND_EXP_MS);
    if (RT_EOK != err) return err;

    cont_blend_reset();
    gpu_lock(DRV_EPIC_FILL_GRAD, NULL, NULL, param);
    drv_epic.cbk = cbk;

    fill_area->x0 = 0;
    fill_area->y0 = 0;
    fill_area->x1 = param->width  - 1;
    fill_area->y1 = param->height - 1;

    if ((HAL_EPIC_AreaWidth(fill_area) > EPIC_COORDINATES_MAX)
            || HAL_EPIC_AreaHeight(fill_area) > EPIC_COORDINATES_MAX)
    {

        EPIC_LayerConfigTypeDef *p_output_canvas = &drv_epic.output_layer;


        HAL_EPIC_LayerConfigInit(p_output_canvas);
        p_output_canvas->color_mode = param->color_mode;
        p_output_canvas->total_width = param->total_width;
        p_output_canvas->data = param->start;
        p_output_canvas->width = param->width;
        p_output_canvas->height = param->height;

        memcpy(drv_epic.grad_color, param->color, sizeof(drv_epic.grad_color));

        drv_epic.split_rd.dst_data = param->start;
        memcpy(&drv_epic.split_rd.render_area, fill_area, sizeof(EPIC_AreaTypeDef));
        drv_epic.split_rd.op = DRV_EPIC_FILL_GRAD;


        ret = epic_split_render_next(1);
    }
    else
    {
        LOG_D("EPIC Fill Grad buf=0x%x, cf=%d, total_w=%d w,h=%d,%d",
              param->start, param->color_mode, param->total_width,
              param->width, param->height);
        LOG_D("Grad color=%08x,%08x,%08x,%08x ",
              param->color[0][0].full, param->color[0][1].full,
              param->color[1][0].full, param->color[1][1].full);


        h_epic->XferCpltCallback = epic_cplt_callback;
        ret = HAL_EPIC_FillGrad_IT(h_epic, param);
    }

    return (HAL_OK == ret) ? RT_EOK : RT_ERROR;
}



rt_err_t drv_epic_blend(EPIC_LayerConfigTypeDef *input_layers,
                        uint8_t input_layer_cnt,
                        EPIC_LayerConfigTypeDef *output_canvas,
                        drv_epic_cplt_cbk cbk)
{

    RT_ASSERT((RT_NULL != output_canvas) && (RT_NULL != input_layers));

    HAL_StatusTypeDef ret;
    rt_err_t err;
    EPIC_HandleTypeDef *h_epic = &epic_handle;
    EPIC_AreaTypeDef *p_blend_area = &drv_epic.split_rd.dst_area;

    err = wait_gpu_done(GPU_BLEND_EXP_MS);
    if (RT_EOK != err) return err;

    cont_blend_reset();
    gpu_lock(DRV_EPIC_IMG_ROT, input_layers, &input_layer_cnt, output_canvas);
    drv_epic.cbk = cbk;

    p_blend_area->x0 = output_canvas->x_offset;
    p_blend_area->y0 = output_canvas->y_offset;
    p_blend_area->x1 = output_canvas->x_offset + output_canvas->width  - 1;
    p_blend_area->y1 = output_canvas->y_offset + output_canvas->height - 1;

    LOG_D("\r\n drv_epic_blend dst=%p "AreaString, output_canvas->data, AreaParams(p_blend_area));


    if ((HAL_EPIC_AreaWidth(p_blend_area) > EPIC_COORDINATES_MAX)
            || HAL_EPIC_AreaHeight(p_blend_area) > EPIC_COORDINATES_MAX)
    {
        if ((input_layer_cnt > 0) && (&drv_epic.input_layers[0] != input_layers))
            memcpy(&drv_epic.input_layers[0], input_layers, input_layer_cnt * sizeof(drv_epic.input_layers[0]));
        if (&drv_epic.output_layer != output_canvas)
            memcpy(&drv_epic.output_layer, output_canvas, sizeof(drv_epic.output_layer));

        drv_epic.input_layer_cnt = input_layer_cnt;
        drv_epic.split_rd.dst_data = output_canvas->data;
        memcpy(&drv_epic.split_rd.render_area, p_blend_area, sizeof(EPIC_AreaTypeDef));
        drv_epic.split_rd.op = DRV_EPIC_IMG_ROT;


        ret = epic_split_render_next(1);
    }
    else
    {
        for (uint8_t i = 0; i < input_layer_cnt; i++)
        {
            PRINT_LAYER_INFO(input_layers + i, "src");
            PRINT_LAYER_EXTRA_INFO(input_layers + i);
        }
        PRINT_LAYER_INFO(output_canvas, "dst");
        h_epic->XferCpltCallback = epic_cplt_callback;
        ret = HAL_EPIC_BlendStartEx_IT(h_epic, input_layers, input_layer_cnt, output_canvas);
    }


    return (HAL_OK == ret) ? RT_EOK : RT_ERROR;

}


rt_err_t drv_epic_transform(EPIC_LayerConfigTypeDef *input_layers,
                            uint8_t input_layer_cnt,
                            EPIC_LayerConfigTypeDef *output_canvas,
                            EPIC_TransPath hor_path,
                            EPIC_TransPath ver_path,
                            void *user_data,
                            drv_epic_cplt_cbk cbk)
{

    RT_ASSERT((RT_NULL != output_canvas) && (RT_NULL != input_layers));

    HAL_StatusTypeDef ret;
    rt_err_t err;
    EPIC_HandleTypeDef *h_epic = &epic_handle;
    EPIC_AreaTypeDef *p_blend_area = &drv_epic.split_rd.dst_area;

    err = wait_gpu_done(GPU_BLEND_EXP_MS);
    if (RT_EOK != err) return err;

    cont_blend_reset();
    gpu_lock(DRV_EPIC_TRANSFORM, input_layers, &input_layer_cnt, output_canvas);
    drv_epic.cbk = cbk;

    p_blend_area->x0 = output_canvas->x_offset;
    p_blend_area->y0 = output_canvas->y_offset;
    p_blend_area->x1 = output_canvas->x_offset + output_canvas->width  - 1;
    p_blend_area->y1 = output_canvas->y_offset + output_canvas->height - 1;

    LOG_D("\r\n drv_epic_transform dst=%p "AreaString, output_canvas->data, AreaParams(p_blend_area));


    if ((HAL_EPIC_AreaWidth(p_blend_area) > EPIC_COORDINATES_MAX)
            || HAL_EPIC_AreaHeight(p_blend_area) > EPIC_COORDINATES_MAX)
    {
        if ((input_layer_cnt > 0) && (&drv_epic.input_layers[0] != input_layers))
            memcpy(&drv_epic.input_layers[0], input_layers, input_layer_cnt * sizeof(drv_epic.input_layers[0]));
        if (&drv_epic.output_layer != output_canvas)
            memcpy(&drv_epic.output_layer, output_canvas, sizeof(drv_epic.output_layer));

        drv_epic.input_layer_cnt = input_layer_cnt;
        drv_epic.split_rd.hor_path = hor_path;
        drv_epic.split_rd.ver_path = ver_path;
        drv_epic.split_rd.user_data = user_data;

        drv_epic.split_rd.dst_data = output_canvas->data;
        memcpy(&drv_epic.split_rd.render_area, p_blend_area, sizeof(EPIC_AreaTypeDef));
        drv_epic.split_rd.op = DRV_EPIC_TRANSFORM;


        ret = epic_split_render_next(1);

        while (DRV_EPIC_INVALID != drv_epic.split_rd.op)
        {
            HAL_StatusTypeDef hal_err = epic_split_render_next(0);
            if (HAL_OK != hal_err)
            {
                drv_epic.split_rd.op = DRV_EPIC_INVALID;
                LOG_E("drv_epic_transform split_render err=%d", hal_err);
                break;
            }
        }

    }
    else
    {
        for (uint8_t i = 0; i < input_layer_cnt; i++)
        {
            PRINT_LAYER_INFO(input_layers + i, "src");
            PRINT_LAYER_EXTRA_INFO(input_layers + i);
        }
        PRINT_LAYER_INFO(output_canvas, "dst");

        ret = HAL_EPIC_TransStart(h_epic, input_layers, input_layer_cnt,
                                  output_canvas, hor_path, ver_path, user_data);

    }

    epic_cplt_callback(h_epic);

    return (HAL_OK == ret) ? RT_EOK : RT_ERROR;

}

rt_err_t drv_epic_cont_blend(EPIC_LayerConfigTypeDef *input_layers,
                             uint8_t input_layer_cnt,
                             EPIC_LayerConfigTypeDef *output_canvas)
{

    RT_ASSERT((RT_NULL != output_canvas) && (RT_NULL != input_layers));

    HAL_StatusTypeDef ret;
    rt_err_t err;
    EPIC_HandleTypeDef *h_epic = &epic_handle;
    EPIC_AreaTypeDef blend_area;
    EPIC_LayerConfigTypeDef *fg_layer = &input_layers[1];
    EPIC_LayerConfigTypeDef *mask_layer = (3 == input_layer_cnt) ? &input_layers[2] : NULL;

    blend_area.x0 = output_canvas->x_offset;
    blend_area.y0 = output_canvas->y_offset;
    blend_area.x1 = output_canvas->x_offset + output_canvas->width  - 1;
    blend_area.y1 = output_canvas->y_offset + output_canvas->height - 1;

    LOG_D("\r\n drv_epic_cont_blend dst=%p "AreaString, output_canvas->data, AreaParams(&blend_area));
    for (uint8_t i = 0; i < input_layer_cnt; i++)
    {
        PRINT_LAYER_INFO(input_layers + i, "src");
        PRINT_LAYER_EXTRA_INFO(input_layers + i);
    }
    PRINT_LAYER_INFO(output_canvas, "dst");

    if (false == drv_epic.cont_mode)
    {
        err = wait_gpu_done(GPU_BLEND_EXP_MS);
        if (RT_EOK != err) return err;

        gpu_lock(DRV_EPIC_LETTER_BLEND, fg_layer, mask_layer, output_canvas);

        ret = HAL_EPIC_ContBlendStart(h_epic, fg_layer, mask_layer, output_canvas);
        drv_epic.cont_mode = true;
    }
    else
    {
        //Clean Dcache
        int dcache_all_cleaned;
        dcache_all_cleaned = mpu_dcache_clean(fg_layer->data, fg_layer->data_size);
        if ((0 == dcache_all_cleaned) && (mask_layer != NULL))
        {
            dcache_all_cleaned = mpu_dcache_clean(mask_layer->data, mask_layer->data_size);
        }

        ret = HAL_EPIC_ContBlendRepeat(h_epic, fg_layer, mask_layer, output_canvas);
    }

    return (HAL_OK == ret) ? RT_EOK : RT_ERROR;
}

void drv_epic_cont_blend_reset(void)
{
    cont_blend_reset();
}

#else
static rt_err_t render_start(void);
static rt_err_t render(drv_epic_render_list_t list, drv_epic_render_cb cb);
static rt_err_t render_end(void);
static rt_err_t destroy_render_list(drv_epic_render_list_t list);

static char *operation_name(drv_epic_op_type_t op)
{
#define OP_TO_NAME_CASE(op) case op: return #op
    switch (op)
    {
        OP_TO_NAME_CASE(DRV_EPIC_COLOR_BLEND);
        OP_TO_NAME_CASE(DRV_EPIC_COLOR_FILL);
        OP_TO_NAME_CASE(DRV_EPIC_LETTER_BLEND);


    default:
        return "UNKNOW";
    }
}


static void print_operation(const char *name, const drv_epic_operation *op)
{


    LOG_E("<<%s, 0x%x,  %s>>", name, op, operation_name(op->op));
    PRINT_AREA_INFO(&op->clip_area, "Clip area");
    if (op->mask.data)
    {
        PRINT_LAYER_INFO(&op->mask, "Mask");
    }

    switch (op->op)
    {
    case DRV_EPIC_COLOR_BLEND:
    {
        PRINT_LAYER_INFO(&op->desc.blend.layer, "FG");
        PRINT_LAYER_EXTRA_INFO(&op->desc.blend.layer);
        if (0 == op->desc.blend.use_dest_as_bg)
        {
            LOG_E("BG rgb:%d,%d,%d",
                  op->desc.blend.r,
                  op->desc.blend.g,
                  op->desc.blend.b);
        }
    }
    break;

    case DRV_EPIC_COLOR_FILL:
    {
        LOG_E("Fill argb:%d,%d,%d,%d",
              op->desc.fill.opa,
              op->desc.fill.r,
              op->desc.fill.g,
              op->desc.fill.b);
    }
    break;


    case DRV_EPIC_LETTER_BLEND:
    {
        LOG_E("Label argb:%d,%d,%d,%d, nums=%d, cf=%d",
              op->desc.label.opa,
              op->desc.label.r,
              op->desc.label.g,
              op->desc.label.b,
              op->desc.label.letter_num,
              op->desc.label.color_mode);

    }
    break;

    default:
    {
        LOG_E("Unknow %d", op->op);
    }
    break;
    }



    LOG_E("\n");
}

static void print_rl(priv_render_list_t *rl)
{
    LOG_E("\n\n\nPrint Render list");
    for (uint16_t i = 0; i < rl->src_list_len; i++)
    {
        drv_epic_operation *p_operation = &rl->src_list[i];
        char idex_str[4];
        rt_sprintf(&idex_str[0], "%d", i);
        print_operation(&idex_str[0], p_operation);
    }

}

#if OUTPUT_TO_GPIO_PIN
extern void BSP_TEST_GPIO_SET(uint32_t pin, uint32_t v);

#define __DEBUG_RENDER_LOCK__(op)      do{if (DRV_EPIC_COLOR_FILL == (ops)){\
                                                BSP_TEST_GPIO_SET(36, 1);\
                                                BSP_TEST_GPIO_SET(36, 0);\
                                            }\
                                            BSP_TEST_GPIO_SET(36, 1);\
                                         }while(0)
#define __DEBUG_RENDER_UNLOCK__  BSP_TEST_GPIO_SET(36, 0)

#define __DEBUG_RENDER_LIST_START__  BSP_TEST_GPIO_SET(35, 1)
#define __DEBUG_RENDER_LIST_END__    BSP_TEST_GPIO_SET(35, 0)

#define __DEBUG_RENDER_LIST_WAIT_EPIC_START__
#define __DEBUG_RENDER_LIST_WAIT_EPIC_END__
#else
#define __DEBUG_RENDER_LOCK__(op)
#define __DEBUG_RENDER_UNLOCK__

#define __DEBUG_RENDER_LIST_START__
#define __DEBUG_RENDER_LIST_END__

#define __DEBUG_RENDER_LIST_WAIT_EPIC_START__
#define __DEBUG_RENDER_LIST_WAIT_EPIC_END__

#endif

static void render_lock(drv_epic_op_type_t ops,
                        uint32_t fg,
                        uint32_t bg,
                        uint32_t mask)
{
    RT_ASSERT((0 == drv_epic.gpu_fg_addr) && (0 == drv_epic.gpu_bg_addr) && (0 == drv_epic.gpu_mask_addr));

    drv_epic.gpu_last_op = ops;
    drv_epic.gpu_fg_addr = fg;
    drv_epic.gpu_bg_addr = bg;
    drv_epic.gpu_mask_addr = mask;

    /* Lock GPU used flash*/
    rt_flash_lock(drv_epic.gpu_fg_addr);

    if (rt_flash_get_handle_by_addr(drv_epic.gpu_fg_addr) != rt_flash_get_handle_by_addr(drv_epic.gpu_bg_addr))
        rt_flash_lock(drv_epic.gpu_bg_addr);

    if ((rt_flash_get_handle_by_addr(drv_epic.gpu_fg_addr) != rt_flash_get_handle_by_addr(drv_epic.gpu_mask_addr))
            && (rt_flash_get_handle_by_addr(drv_epic.gpu_bg_addr) != rt_flash_get_handle_by_addr(drv_epic.gpu_mask_addr)))
        rt_flash_lock(drv_epic.gpu_mask_addr);



    __DEBUG_RENDER_LOCK__(ops);

}
static void render_unlock(void)
{

    __DEBUG_RENDER_UNLOCK__;


    /* UnLock GPU used flash*/
    rt_flash_unlock(drv_epic.gpu_fg_addr);

    if (rt_flash_get_handle_by_addr(drv_epic.gpu_fg_addr) != rt_flash_get_handle_by_addr(drv_epic.gpu_bg_addr))
        rt_flash_unlock(drv_epic.gpu_bg_addr);

    if ((rt_flash_get_handle_by_addr(drv_epic.gpu_fg_addr) != rt_flash_get_handle_by_addr(drv_epic.gpu_mask_addr))
            && (rt_flash_get_handle_by_addr(drv_epic.gpu_bg_addr) != rt_flash_get_handle_by_addr(drv_epic.gpu_mask_addr)))
        rt_flash_unlock(drv_epic.gpu_mask_addr);




    drv_epic.gpu_fg_addr  = 0;
    drv_epic.gpu_bg_addr  = 0;
    drv_epic.gpu_mask_addr = 0;

    drv_epic.gpu_output_addr = 0;
    drv_epic.gpu_output_size = 0;
}

rt_err_t render_start(void)
{
#ifdef BSP_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif /*BSP_USING_PM*/
    GPU_DEVICE_START();
#ifdef PSRAM_CACHE_WB
    SCB_CleanDCache();
#endif

    return RT_EOK;
}


rt_err_t render_end(void)
{

    SCB_InvalidateDCache();


    GPU_DEVICE_STOP();
#ifdef BSP_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif /*BSP_USING_PM*/

    return RT_EOK;
}

static void epic_cplt_callback(EPIC_HandleTypeDef *epic)
{
    epic_cbk_ctx_t *p_ctx = (epic_cbk_ctx_t *) epic->user_data;
    drv_epic_render_cb cb = p_ctx->cb;
    EPIC_LayerConfigTypeDef *p_dst = &p_ctx->rl->dst;

    render_unlock();
    epic_sem_release();

#if debug_rl_hist_num > 0
    p_ctx->p_hist->cost_us += GetElapsedUs(p_ctx->p_hist->start_tick, HAL_DBG_DWT_GetCycles());
#endif /* debug_rl_hist_num > 0 */

    if (cb) cb(p_dst);
}

static void pixel_offset(uint32_t offset_bytes, uint32_t line_width, uint32_t color_bytes, int16_t *x_off, int16_t *y_off)
{
    RT_ASSERT(0 == (offset_bytes % color_bytes));
    uint32_t offset_pixels = offset_bytes / color_bytes;

    *y_off = offset_pixels / line_width;
    *x_off = offset_pixels - (line_width * (*y_off));
}

static inline int16_t merge_a2b(
    const drv_epic_render_buf *p_buf_a,
    const EPIC_LayerConfigTypeDef *p_buf_b,
    int16_t *x_off, //Vector of a->b
    int16_t *y_off,
    uint16_t *new_height)
{
    if (p_buf_a->cf != p_buf_b->color_mode) return 0;
    if (p_buf_a->area.x1 - p_buf_a->area.x0 + 1 != p_buf_b->total_width) return 0;

    uint32_t color_bytes = HAL_EPIC_GetColorDepth(p_buf_b->color_mode) >> 3;
    RT_ASSERT(color_bytes > 0);

    uint32_t p_buf_a_bytes = HAL_EPIC_AreaWidth(&p_buf_a->area) * HAL_EPIC_AreaHeight(&p_buf_a->area) * color_bytes;
    uint32_t p_buf_b_bytes = p_buf_b->total_width * p_buf_b->height * color_bytes;

    if ((p_buf_a->data == p_buf_b->data) && (p_buf_a_bytes == p_buf_b_bytes))
    {
        *x_off = 0;
        *y_off = 0;
        return 1;//Same buffer
    }
    else if (p_buf_a->data <= p_buf_b->data)
    {
        if ((p_buf_a->data + p_buf_a_bytes) >= (p_buf_b->data + p_buf_b_bytes))
        {
            pixel_offset(p_buf_b->data - p_buf_a->data, p_buf_b->total_width, color_bytes, x_off, y_off);

            return 2;//A include B
        }
        else if ((p_buf_a->data + p_buf_a_bytes) >= p_buf_b->data)
        {
            pixel_offset(p_buf_b->data - p_buf_a->data, p_buf_b->total_width, color_bytes, x_off, y_off);

            if (*x_off)
            {
                return 0;//Can't merge to 1 buffer
            }
            else
            {
                *new_height = p_buf_b->height + *y_off;
                return 3;//A&B have intersection
            }
        }
        else
        {
            return 0;//No intersection
        }
    }
    else
    {
        if ((p_buf_b->data + p_buf_b_bytes) >= (p_buf_a->data + p_buf_a_bytes))
        {
            pixel_offset(p_buf_a->data - p_buf_b->data, p_buf_b->total_width, color_bytes, x_off, y_off);
            *y_off = - *y_off;
            *x_off = - *x_off;
            return 4;//B include A
        }
        else if ((p_buf_b->data + p_buf_b_bytes) >= p_buf_a->data)
        {
            pixel_offset(p_buf_a->data - p_buf_b->data, p_buf_b->total_width, color_bytes, x_off, y_off);

            if (*x_off)
            {
                return 0;//Can't merge to 1 buffer
            }
            else
            {
                *new_height = HAL_EPIC_AreaHeight(&p_buf_a->area) + *y_off;
                *y_off = - *y_off;
                *x_off = - *x_off;
                return 5;//B&A have intersection
            }
        }
        else
        {
            return 0; //No intersection
        }
    }


}


static rt_err_t lcd_flush_done(rt_device_t dev, void *buffer)
{
    drv_epic.lcd_te = 0;
    rt_device_control(dev, RTGRAPHIC_CTRL_SET_NEXT_TE, &drv_epic.lcd_te);
    dev->tx_complete = drv_epic.lcd_tx_complete;
    rt_sem_release(&drv_epic.lcd_sema);
    return RT_EOK;
}

static void lcd_flush(rt_device_t dev, const EPIC_LayerConfigTypeDef *dst)
{
    EPIC_AreaTypeDef  area;


    area.x0 = dst->x_offset;
    area.x1 = dst->x_offset + dst->width - 1;
    area.y0 = dst->y_offset;
    area.y1 = dst->y_offset + dst->height - 1;

    rt_graphix_ops(dev)->set_window(area.x0, area.y0, area.x1, area.y1);
    rt_graphix_ops(dev)->draw_rect_async((const char *)dst->data, area.x0, area.y0, area.x1, area.y1);
}

static void wait_lcd_flush_done(EPIC_DrvTypeDef *p_drv_epic)
{
    rt_err_t err;

    uint32_t start_ms = rt_tick_get_millisecond();
    err = rt_sem_take(&p_drv_epic->lcd_sema, rt_tick_from_millisecond(1000));
    RT_ASSERT(RT_EOK == err);
    drv_epic.rd_flush_lcd_wait  += rt_tick_get_millisecond() - start_ms;
}

static void wait_hw_done(void)
{
    __DEBUG_RENDER_LIST_WAIT_EPIC_START__;

    if (RT_EOK != drv_gpu_take(GPU_BLEND_EXP_MS))
    {
#ifdef __RELEASE__
        gpu_reset();
#else
        RT_ASSERT(0); //Raise an assertion in debug mode.
#endif /* __RELEASE__ */
    }

    __DEBUG_RENDER_LIST_WAIT_EPIC_END__;

}
static rt_err_t render(drv_epic_render_list_t list, drv_epic_render_cb cb)
{
    EPIC_AreaTypeDef dst_area, intersect_area;
    priv_render_list_t *rl = (priv_render_list_t *) list;
    EPIC_LayerConfigTypeDef *dst = &rl->dst;
    uint32_t dst_color_bytes = HAL_EPIC_GetColorDepth(dst->color_mode) >> 3;
    rt_err_t ret_v = RT_EEMPTY;

    for (uint16_t i = 0; i < rl->src_list_len; i++)
    {
        drv_epic_operation *p_operation = &rl->src_list[i];
        HAL_StatusTypeDef ret;

        dst_area.x0 = dst->x_offset + p_operation->offset_x;
        dst_area.y0 = dst->y_offset + p_operation->offset_y;
        dst_area.x1 = dst_area.x0 + dst->width - 1;
        dst_area.y1 = dst_area.y0 + dst->height - 1;

        if (HAL_EPIC_AreaIntersect(&intersect_area, &dst_area, &p_operation->clip_area))
        {
            EPIC_LayerConfigTypeDef output_layer;
            uint32_t mask_addr = 0;
            uint32_t fg_addr = 0;
            uint32_t bg_addr = 0;


#if !defined(EPIC_SUPPORT_MASK)
            RT_ASSERT(NULL == p_operation->mask.data);
#endif


            if ((1 == drv_epic.dbg_flag_dis_ram_instance)
                    || (DRV_EPIC_LETTER_BLEND == p_operation->op))
            {
                wait_hw_done();
                epic_sem_release();
            }


#if debug_rl_hist_num > 0
            priv_render_hist_t *p_cur_hist;
            drv_epic.hist_idx = (drv_epic.hist_idx + 1) % debug_rl_hist_num;
            p_cur_hist = &drv_epic.hist[drv_epic.hist_idx];
            p_cur_hist->rl = rl;
            p_cur_hist->src_idx = i;
            p_cur_hist->start_tick = HAL_DBG_DWT_GetCycles();
            p_cur_hist->cost_us = 0;
#endif /* debug_rl_hist_num > 0 */


            if (DRV_EPIC_LETTER_BLEND == p_operation->op)
            {
                drv_epic.using_epic = drv_get_epic_handle();
                drv_epic.using_epic->XferCpltCallback = NULL;
#if debug_rl_hist_num > 0
                p_cur_hist->use_ram_epic = 0;
#endif

                {
                    HAL_StatusTypeDef ret;
                    EPIC_LayerConfigTypeDef fg_layer, mask_layer;
                    uint8_t input_layer_cnt = 2;
                    uint8_t cont_blend_first = 1;

                    RT_ASSERT(&epic_handle == drv_epic.using_epic);

                    if (p_operation->mask.data)
                    {
                        //Setup mask layer
                        memcpy(&mask_layer, &p_operation->mask, sizeof(EPIC_LayerConfigTypeDef));
                        mask_addr = (uint32_t) p_operation->mask.data;
                        input_layer_cnt++;
                    }

                    /*Inital fg layer*/
                    HAL_EPIC_LayerConfigInit(&fg_layer);
                    fg_layer.alpha = p_operation->desc.label.opa;
                    fg_layer.color_en = true;
                    fg_layer.color_r = p_operation->desc.label.r;
                    fg_layer.color_g = p_operation->desc.label.g;
                    fg_layer.color_b = p_operation->desc.label.b;
                    fg_layer.color_mode = p_operation->desc.label.color_mode;
                    fg_layer.ax_mode = ALPHA_BLEND_RGBCOLOR;
                    /*Inital output layer*/
                    memcpy(&output_layer, dst, sizeof(EPIC_LayerConfigTypeDef));

                    for (uint32_t i = 0; i < p_operation->desc.label.letter_num; i++)
                    {
                        drv_epic_letter_type_t *p_letter = p_operation->desc.label.p_letters + i;
                        EPIC_AreaTypeDef final_area;

                        if (HAL_EPIC_AreaIntersect(&final_area, &intersect_area, &p_letter->area))
                        {
                            //setup fg
                            fg_layer.data = (uint8_t *)p_letter->data;
                            fg_layer.width  = HAL_EPIC_AreaWidth(&p_letter->area);
                            fg_layer.height = HAL_EPIC_AreaHeight(&p_letter->area);
                            fg_layer.total_width = fg_layer.width;
                            fg_layer.x_offset = p_letter->area.x0;
                            fg_layer.y_offset = p_letter->area.y0;
                            fg_addr = (uint32_t)fg_layer.data;


                            //setup output_layer
                            {

                                int16_t x_off = final_area.x0 - dst_area.x0;
                                int16_t y_off = final_area.y0 - dst_area.y0;


                                output_layer.data     = dst->data + (y_off * dst->total_width + x_off) * dst_color_bytes;
                                output_layer.x_offset = dst_area.x0 + x_off;
                                output_layer.y_offset = dst_area.y0 + y_off;
                                output_layer.width    = final_area.x1 - final_area.x0 + 1;
                                output_layer.height   = final_area.y1 - final_area.y0 + 1;
                            }




                            if (cont_blend_first)
                            {
                                cont_blend_first = 0;

                                render_lock(DRV_EPIC_LETTER_BLEND, fg_addr, bg_addr, mask_addr); //Assume all data at same memory
                                ret = HAL_EPIC_ContBlendStart(drv_epic.using_epic, &fg_layer,
                                                              (3 == input_layer_cnt) ? &mask_layer : NULL,
                                                              &output_layer);

                            }
                            else
                            {
                                ret = HAL_EPIC_ContBlendRepeat(drv_epic.using_epic, &fg_layer,
                                                               (3 == input_layer_cnt) ? &mask_layer : NULL,
                                                               &output_layer);
                            }
                            RT_ASSERT(HAL_OK == ret);
                        }
                    }

                    if (!cont_blend_first)
                    {
                        ret = HAL_EPIC_ContBlendStop(drv_epic.using_epic);
                        RT_ASSERT(HAL_OK == ret);
                        render_unlock();
                    }
                }

#if debug_rl_hist_num > 0
                p_cur_hist->cost_us = GetElapsedUs(p_cur_hist->start_tick, HAL_DBG_DWT_GetCycles());
#endif /* debug_rl_hist_num > 0 */

                if (i == rl->src_list_len)
                {
                    if (cb) cb(dst);
                }
            }
            else
            {
                //Clip output_layer
                memcpy(&output_layer, dst, sizeof(EPIC_LayerConfigTypeDef));

                int16_t x_off = intersect_area.x0 - dst_area.x0;
                int16_t y_off = intersect_area.y0 - dst_area.y0;


                output_layer.data     += (y_off * dst->total_width + x_off) * dst_color_bytes;
                output_layer.x_offset = dst_area.x0 + x_off;
                output_layer.y_offset = dst_area.y0 + y_off;
                output_layer.width    = intersect_area.x1 - intersect_area.x0 + 1;
                output_layer.height   = intersect_area.y1 - intersect_area.y0 + 1;


                if (RT_EOK == epic_sem_trytake())
                {
                    drv_epic.using_epic = drv_get_epic_handle();
                    if (i == rl->src_list_len) drv_epic.epic_cb_ctx.cb = cb;
                    drv_epic.using_epic->user_data = &drv_epic.epic_cb_ctx;
#if debug_rl_hist_num > 0
                    p_cur_hist->use_ram_epic = 0;
                    drv_epic.epic_cb_ctx.p_hist = p_cur_hist;
#endif
                    drv_epic.using_epic->XferCpltCallback = epic_cplt_callback;
                }
                else
                {
                    HAL_EPIC_BlendFastStart_Init(&drv_epic.epic_handle2);
                    drv_epic.using_epic = &drv_epic.epic_handle2;
                    drv_epic.using_epic->XferCpltCallback = NULL;
                }


                switch (p_operation->op)
                {
                case DRV_EPIC_COLOR_BLEND:
                {
                    EPIC_LayerConfigTypeDef input_layers[3];

                    uint8_t input_layer_cnt = 2;

                    memcpy(&input_layers[0], dst, sizeof(EPIC_LayerConfigTypeDef));
                    memcpy(&input_layers[1], &p_operation->desc.blend.layer, sizeof(EPIC_LayerConfigTypeDef));
                    fg_addr = (uint32_t) p_operation->desc.blend.layer.data;
                    if (p_operation->mask.data)
                    {
                        memcpy(&input_layers[2], &p_operation->mask, sizeof(EPIC_LayerConfigTypeDef));
                        mask_addr = (uint32_t) p_operation->mask.data;
                        input_layer_cnt++;
                    }

                    if (fg_addr == drv_epic.dbg_src_addr)
                    {
                        __asm__("B .");
                    }

                    if (&drv_epic.epic_handle2 != drv_epic.using_epic) render_lock(DRV_EPIC_COLOR_BLEND, fg_addr, bg_addr, mask_addr);
                    if (0 == p_operation->desc.blend.use_dest_as_bg)
                    {
                        output_layer.color_en = true;
                        output_layer.color_r  = p_operation->desc.blend.r;
                        output_layer.color_g  = p_operation->desc.blend.g;
                        output_layer.color_b  = p_operation->desc.blend.b;
                        ret = HAL_EPIC_BlendStartEx_IT(drv_epic.using_epic, &input_layers[1], input_layer_cnt - 1, &output_layer);
                    }
                    else
                    {
                        ret = HAL_EPIC_BlendStartEx_IT(drv_epic.using_epic, input_layers, input_layer_cnt, &output_layer);
                    }
                    RT_ASSERT(HAL_OK == ret);
                }
                break;

                case DRV_EPIC_COLOR_FILL:
                {
                    output_layer.color_r = p_operation->desc.fill.r;
                    output_layer.color_g = p_operation->desc.fill.g;
                    output_layer.color_b = p_operation->desc.fill.b;

                    if (p_operation->mask.data)
                    {
                        EPIC_LayerConfigTypeDef input_layers[3];

                        memcpy(&input_layers[2], &p_operation->mask, sizeof(EPIC_LayerConfigTypeDef));

                        input_layers[1] = output_layer;
                        input_layers[1].data = (uint8_t *) input_layers;//Any accessable address for mono layer
                        input_layers[1].color_mode = EPIC_INPUT_MONO;
                        input_layers[1].alpha = p_operation->desc.fill.opa;
                        input_layers[1].ax_mode = ALPHA_BLEND_RGBCOLOR;
                        input_layers[1].color_en = true;
                        mask_addr = (uint32_t) p_operation->mask.data;


                        memcpy(&input_layers[0], dst, sizeof(EPIC_LayerConfigTypeDef));
                        if (&drv_epic.epic_handle2 != drv_epic.using_epic) render_lock(DRV_EPIC_COLOR_FILL, fg_addr, bg_addr, mask_addr);
                        ret = HAL_EPIC_BlendStartEx_IT(drv_epic.using_epic, input_layers, 3, &output_layer);
                    }
                    else if (p_operation->desc.fill.opa != 255)
                    {
                        EPIC_LayerConfigTypeDef input_layer = output_layer;

                        uint8_t opa = p_operation->desc.fill.opa;
                        input_layer.alpha = (0 == opa) ? 255 : (256 - opa);

                        output_layer.color_en = true;
                        ret = HAL_EPIC_BlendStartEx_IT(drv_epic.using_epic, &input_layer, 1, &output_layer);
                    }
                    else
                    {
                        output_layer.color_en = true;
                        ret = HAL_EPIC_BlendStartEx_IT(drv_epic.using_epic, NULL, 0, &output_layer);
                    }
                    RT_ASSERT(HAL_OK == ret);

                }
                break;


                default:
                    RT_ASSERT(0);
                    break;

                }

                //Using shadow handler and having sth. to do(State is busy)
                if ((&drv_epic.epic_handle2 == drv_epic.using_epic)
                        && (HAL_EPIC_STATE_BUSY == drv_epic.using_epic->State))
                {
#if debug_rl_hist_num > 0
                    p_cur_hist->cost_us += GetElapsedUs(p_cur_hist->start_tick, HAL_DBG_DWT_GetCycles()); //Skip wait hw done time.
#endif
                    wait_hw_done();
#if debug_rl_hist_num > 0
                    p_cur_hist->start_tick = HAL_DBG_DWT_GetCycles(); //Reset start time
#endif

                    render_lock(p_operation->op, fg_addr, bg_addr, mask_addr);
                    drv_epic.using_epic->XferCpltCallback = epic_cplt_callback;
                    if (i == rl->src_list_len) drv_epic.epic_cb_ctx.cb = cb;
                    drv_epic.using_epic->user_data = &drv_epic.epic_cb_ctx;
#if debug_rl_hist_num > 0
                    p_cur_hist->use_ram_epic = 1;
                    drv_epic.epic_cb_ctx.p_hist = p_cur_hist;
#endif
                    ret = HAL_EPIC_BlendFastStart_IT(drv_get_epic_handle(), &drv_epic.epic_handle2);
                    RT_ASSERT(HAL_OK == ret);
                }

            }

            ret_v = RT_EOK;

        }
    }


    return ret_v;
}

static rt_err_t render_list(priv_render_list_t *rl)
{
    __DEBUG_RENDER_LIST_START__;

    rt_err_t ret;

    ret = render((drv_epic_render_list_t)rl, NULL);


    drv_gpu_check_done(GPU_BLEND_EXP_MS);
    __DEBUG_RENDER_LIST_END__;

    return ret;
}



rt_err_t destroy_render_list(drv_epic_render_list_t list)
{
    priv_render_list_t *rl = (priv_render_list_t *) list;

    rt_enter_critical();
    rl->flag = 0;
    rl->src_list_len = 0;
    rl->letter_pool_free = 0;
    rl->used = 0;
    rl->commit_area.x0 = 0;
    rl->commit_area.x1 = -1;
    rl->commit_area.y0 = 0;
    rl->commit_area.y1 = -1;
    rt_exit_critical();
    return RT_EOK;
}

static void epic_task(void *param)
{
    EPIC_DrvTypeDef *p_drv_epic = (EPIC_DrvTypeDef *)param;

    EPIC_MsgTypeDef msg;
    rt_err_t err;

    while (1)
    {
        err = rt_mq_recv(p_drv_epic->mq, &msg, sizeof(msg), RT_WAITING_FOREVER);

        RT_ASSERT(RT_EOK == err);

        LOG_D("epic_task exec msg%x: [%d].", msg.tick, msg.id);


        switch (msg.id)
        {
        case EPIC_MSG_RENDER_DRAW:
        {
            drv_epic_render_draw_cfg *p_RenderDrawctx = &msg.content.rd;
            EPIC_AreaTypeDef *p_invalid_area = &p_RenderDrawctx->area;
            uint32_t max_row;


            uint32_t start_ms = rt_tick_get_millisecond();
            uint32_t start_epic_cnt = epic_handle.PerfCnt;

            /*Start flush*/
            priv_render_list_t *rl = (priv_render_list_t *)p_RenderDrawctx->render_list;
            EPIC_LayerConfigTypeDef *p_dst = &rl->dst;
            rt_enter_critical();
            if (0 == (rl->flag & rl_flag_overwritting))
                rl->flag |= rl_flag_rendering;
            else
                rl->flag &= ~rl_flag_commit;
            rt_exit_critical();

            if (rl->flag & rl_flag_rendering)
            {
                uint32_t color_bytes = HAL_EPIC_GetColorDepth(p_RenderDrawctx->cf) >> 3;
                HAL_EPIC_LayerConfigInit(p_dst);
                p_dst->color_mode = p_RenderDrawctx->cf;
                p_dst->width = p_invalid_area->x1 - p_invalid_area->x0 + 1;
                p_dst->total_width = p_dst->width;
                max_row = (uint32_t)(p_RenderDrawctx->buf_bytes / color_bytes) / p_dst->total_width;
                p_dst->height = RT_ALIGN_DOWN(max_row, p_RenderDrawctx->pixel_align);
                p_dst->data_size = color_bytes * p_dst->total_width * p_dst->height;
                p_dst->x_offset = p_invalid_area->x0;
                p_dst->data = (uint8_t *)p_RenderDrawctx->buf1;

                render_start();
                rt_device_t p_lcd_device = p_RenderDrawctx->lcd_dev;

                drv_epic.lcd_tx_complete = p_lcd_device->tx_complete;

                if (drv_epic.dbg_flag_print_rl) print_rl(rl);

                for (int16_t start_row = p_invalid_area->y0; start_row <= p_invalid_area->y1; start_row += p_dst->height)
                {
                    //lv_area_t buf_area;
                    p_dst->y_offset = start_row;
                    if (start_row + p_dst->height - 1 > p_invalid_area->y1)
                    {
                        p_dst->height = p_invalid_area->y1 - start_row + 1;
                    }

                    if (RT_EOK == render_list(rl))
                    {

                        uint32_t lcd_start_ms = rt_tick_get_millisecond();

                        wait_lcd_flush_done(p_drv_epic);

                        rt_device_set_tx_complete(p_lcd_device, lcd_flush_done);

                        lcd_flush(p_lcd_device, p_dst);

                        p_drv_epic->rd_lcd_total += rt_tick_get_millisecond() - lcd_start_ms;

                        if (p_dst->data == (uint8_t *)p_RenderDrawctx->buf1)
                            p_dst->data = (uint8_t *)p_RenderDrawctx->buf2;
                        else
                            p_dst->data = (uint8_t *)p_RenderDrawctx->buf1;
                    }
                }

                destroy_render_list(p_RenderDrawctx->render_list);
                render_end();

                wait_lcd_flush_done(p_drv_epic);
                rt_sem_release(&p_drv_epic->lcd_sema);
                p_drv_epic->lcd_te = 1;
                rt_device_control(p_lcd_device, RTGRAPHIC_CTRL_SET_NEXT_TE, &p_drv_epic->lcd_te);

                {
                    uint32_t end_ms = rt_tick_get_millisecond();
                    uint32_t cost_ms = end_ms - start_ms;

                    p_drv_epic->rd_count++;
                    p_drv_epic->rd_total += cost_ms;

                    p_drv_epic->rd_min = MIN(p_drv_epic->rd_min, cost_ms);
                    p_drv_epic->rd_max = MAX(p_drv_epic->rd_max, cost_ms);
                    p_drv_epic->rd_epic_hw_us += GetElapsedUs(start_epic_cnt, epic_handle.PerfCnt);

                    if (p_drv_epic->rd_total > 2000)
                    {
                        LOG_E("Rend&DrawLCD(ms) min:%d max:%d avg:%d, DrawLCD(ms) avg:%d",
                              p_drv_epic->rd_min, p_drv_epic->rd_max,
                              p_drv_epic->rd_total / p_drv_epic->rd_count,
                              p_drv_epic->rd_lcd_total / p_drv_epic->rd_count);

                        LOG_E("Times(ms) Total(%d)=wait_lcd(%d)+wait_epic(%d)+sw(%d), epic_hw:%d%%",
                              p_drv_epic->rd_total,
                              p_drv_epic->rd_flush_lcd_wait, p_drv_epic->rd_epic_wait,
                              p_drv_epic->rd_total - p_drv_epic->rd_flush_lcd_wait - p_drv_epic->rd_epic_wait,
                              (p_drv_epic->rd_epic_hw_us / 10) / p_drv_epic->rd_total
                             );

                        p_drv_epic->rd_count = 0;
                        p_drv_epic->rd_min = UINT32_MAX;
                        p_drv_epic->rd_max = 0;
                        p_drv_epic->rd_total = 0;
                        p_drv_epic->rd_lcd_total = 0;

                        p_drv_epic->rd_flush_lcd_wait = 0;
                        p_drv_epic->rd_epic_wait = 0;
                        p_drv_epic->rd_epic_hw_us = 0;
                    }
                }
            }
        }
        break;




        default:
            RT_ASSERT(0);
            break;
        }

    }

}


#if 0

static void update_list_op_offset(priv_render_list_t *rl,
                                  int16_t x_off, int16_t y_off //Vector from old to new
                                 )
{
    for (uint16_t idx = 0; idx < rl->src_list_len; idx++)
    {
        rl->src_list[idx].offset_x += x_off;
        rl->src_list[idx].offset_y += y_off;
    }
}

void test_merge(void)
{
    int16_t merge_ret;
    int16_t x_off, y_off; //Vector from 'p_buf(A)' to 'rl->dst(B)'
    uint16_t new_height;

    drv_epic_render_buf request_buf;
    EPIC_LayerConfigTypeDef dst;



    //Initl

    request_buf.cf = EPIC_COLOR_RGB565;
    request_buf.data = (uint8_t *)0x10000000;
    request_buf.area.x0 = 100;
    request_buf.area.y0 = 100;
    request_buf.area.x1 = 199;
    request_buf.area.y1 = 199;

    HAL_EPIC_LayerConfigInit(&dst);
    dst.data        = request_buf.data;
    dst.color_mode  = request_buf.cf;
    dst.width       = HAL_EPIC_AreaWidth(&request_buf.area);
    dst.height      = HAL_EPIC_AreaHeight(&request_buf.area);
    dst.total_width = dst.width;
    dst.x_offset    = request_buf.area.x0;
    dst.y_offset    = request_buf.area.y0;


    merge_ret = merge_a2b(&request_buf, &dst, &x_off, &y_off, &new_height);
    RT_ASSERT(1 == merge_ret);


    request_buf.data = (uint8_t *)0x10000000 - 400;
    merge_ret = merge_a2b(&request_buf, &dst, &x_off, &y_off, &new_height);
    RT_ASSERT(3 == merge_ret); //A&B
    RT_ASSERT(2 == y_off);
    RT_ASSERT(102 == new_height);


    request_buf.data = (uint8_t *)0x10000000 + 400;
    merge_ret = merge_a2b(&request_buf, &dst, &x_off, &y_off, &new_height);
    RT_ASSERT(5 == merge_ret);//B&A
    RT_ASSERT(-2 == y_off);
    RT_ASSERT(102 == new_height);


    request_buf.data = (uint8_t *)0x10000000 - 400 - 4;
    request_buf.area.y0 = 100;
    request_buf.area.y1 = 299;
    merge_ret = merge_a2b(&request_buf, &dst, &x_off, &y_off, &new_height);
    RT_ASSERT(2 == merge_ret);//A include B
    RT_ASSERT(2 == x_off);
    RT_ASSERT(2 == y_off);



    request_buf.data = (uint8_t *)0x10000000 + 400 + 4;
    request_buf.area.y0 = 111;
    request_buf.area.y1 = 199;
    merge_ret = merge_a2b(&request_buf, &dst, &x_off, &y_off, &new_height);
    RT_ASSERT(4 == merge_ret);//B include A
    RT_ASSERT(-2 == x_off);
    RT_ASSERT(-2 == y_off);


}
#endif /* 0 */

/**
 * Get the rectangle area that substract an area form another
 * @param aex_p pointer to an area which should not in `aholder_p`
 * @param aholder_p pointer to an area which we considered
 * @return true: `aex_p` is fully inside `aholder_p`, and the area else is a rectangle area.
 */
static bool AreaExcept(EPIC_AreaTypeDef *res_p, const EPIC_AreaTypeDef *aex_p, const EPIC_AreaTypeDef *aholder_p)
{
    if ((aholder_p->x0 == aex_p->x0) && (aholder_p->x1 == aex_p->x1))
    {
        if ((aholder_p->y0 == aex_p->y0) && (aholder_p->y1 > aex_p->y1))
        {
            res_p->x0 = aholder_p->x0;
            res_p->x1 = aholder_p->x1;
            res_p->y0 = aex_p->y1 + 1;
            res_p->y1 = aholder_p->y1;
            return true;
        }
        else if ((aholder_p->y0 < aex_p->y0) && (aholder_p->y1 == aex_p->y1))
        {
            res_p->x0 = aholder_p->x0;
            res_p->x1 = aholder_p->x1;
            res_p->y0 = aholder_p->y0;
            res_p->y1 = aex_p->y0 - 1;
            return true;
        }
    }
    else if ((aholder_p->y0 == aex_p->y0) && (aholder_p->y1 == aex_p->y1))
    {
        if ((aholder_p->x0 == aex_p->x0) && (aholder_p->x1 > aex_p->x1))
        {
            res_p->x0 = aex_p->x1 + 1;
            res_p->x1 = aholder_p->x1;
            res_p->y0 = aholder_p->y0;
            res_p->y1 = aholder_p->y1;
            return true;
        }
        else if ((aholder_p->x0 < aex_p->x0) && (aholder_p->x1 == aex_p->x1))
        {
            res_p->x0 = aholder_p->x0;
            res_p->x1 = aex_p->x0 - 1;
            res_p->y0 = aholder_p->y0;
            res_p->y1 = aholder_p->y1;
            return true;
        }
    }

    return false;
}

drv_epic_render_list_t drv_epic_alloc_render_list(drv_epic_render_buf *p_buf, EPIC_AreaTypeDef *p_ow_area)
{
    priv_render_list_t *rl_overwrite = NULL;
    priv_render_list_t *rl_ret = NULL;

    rt_enter_critical();

    for (uint32_t idx = 0; idx < render_list_pool_max; idx++)
    {
        priv_render_list_t *rl = &drv_epic.render_list_pool[idx];
        if (0 == rl->used)
        {
            RT_ASSERT(0 == rl->flag);
            RT_ASSERT(0 == rl->src_list_len);

            rl->used = 1;
            rl_ret = rl;
            break;
        }
        else if (0 == (rl_flag_rendering & rl->flag))
        {
            rl->src_list_len = 0;
            rl->flag |= rl_flag_overwritting;
            rl_overwrite = rl;
        }
    }
    rt_exit_critical();

    if (rl_overwrite && !rl_ret) rl_ret = rl_overwrite;

    if (rl_ret)
    {
        HAL_EPIC_LayerConfigInit(&rl_ret->dst);
        rl_ret->dst.data        = p_buf->data;
        rl_ret->dst.color_mode  = p_buf->cf;
        rl_ret->dst.width       = HAL_EPIC_AreaWidth(&p_buf->area);
        rl_ret->dst.height      = HAL_EPIC_AreaHeight(&p_buf->area);
        rl_ret->dst.total_width = rl_ret->dst.width;
        rl_ret->dst.x_offset    = p_buf->area.x0;
        rl_ret->dst.y_offset    = p_buf->area.y0;

        HAL_EPIC_AreaCopy(p_ow_area, &rl_ret->commit_area);
    }

    drv_epic.using_rl = rl_ret;
    return (drv_epic_render_list_t)rl_ret;
}

drv_epic_operation *drv_epic_alloc_op(drv_epic_render_buf *p_buf)
{
    priv_render_list_t *found_list = NULL;
    priv_render_list_t *free_list = NULL;
    int16_t merge_ret;
    int16_t x_off, y_off; //Vector from 'p_buf(A)' to 'rl->dst(B)'
    uint16_t new_height;
    priv_render_list_t *rl;

    //test_merge();

#if 0
    //Try to find same rendering list
    for (uint32_t idx = 0; idx < render_list_pool_max; idx++)
    {
        rl = &drv_epic.render_list_pool[idx];
        if (0 == (rl->flag & rl_flag_rendering))
        {
            if (rl->used)
            {
                merge_ret = merge_a2b(p_buf, &rl->dst, &x_off, &y_off, &new_height);
                if (merge_ret > 0)
                {
                    found_list = rl;
                    break;
                }
            }
            else if (!free_list)
            {
                free_list = rl;
            }
        }
    }
#else
    rl = drv_epic.using_rl;
    RT_ASSERT(rl);
    RT_ASSERT(0 == (rl->flag & rl_flag_rendering));

    if (rl->src_list_len > 0)
    {
        found_list = rl;
        merge_ret = merge_a2b(p_buf, &rl->dst, &x_off, &y_off, &new_height);
    }
    else
    {
        free_list = rl;
    }
#endif

    //Update the dst layer of rendering list
    if (found_list)
    {
        rl = found_list;

        switch (merge_ret)
        {
        case 2://A include B
            rl->dst.data        = p_buf->data;
            rl->dst.height      = HAL_EPIC_AreaHeight(&p_buf->area);
            rl->dst.x_offset    = rl->dst.x_offset - x_off;
            rl->dst.y_offset    = rl->dst.y_offset - y_off;
            break;

        case 3://A&B have intersection
            rl->dst.data        = p_buf->data;
            rl->dst.height      = new_height;
            rl->dst.x_offset    = rl->dst.x_offset - x_off;
            rl->dst.y_offset    = rl->dst.y_offset - y_off;
            break;

        case 5://B&A have intersection
            rl->dst.height      = new_height;
            break;

        case 1: //A == B
        case 4: //B include A
            break;

        default:
            RT_ASSERT(0);
            break;
        }
    }
    else if (free_list)
    {
        rl = free_list;

        HAL_EPIC_LayerConfigInit(&rl->dst);
        rl->dst.data        = p_buf->data;
        rl->dst.color_mode  = p_buf->cf;
        rl->dst.width       = HAL_EPIC_AreaWidth(&p_buf->area);
        rl->dst.height      = HAL_EPIC_AreaHeight(&p_buf->area);
        rl->dst.total_width = rl->dst.width;
        rl->dst.x_offset    = p_buf->area.x0;
        rl->dst.y_offset    = p_buf->area.y0;
    }
    else
    {
        return NULL;
    }




    if (rl->src_list_len < src_list_max)
    {
        drv_epic_operation *ret_op;
        ret_op = &rl->src_list[rl->src_list_len];

        if ((rl == found_list) && ((4 == merge_ret) || (5 == merge_ret)))
        {
            ret_op->offset_x = x_off;
            ret_op->offset_y = y_off;
        }
        else
        {
            ret_op->offset_x = 0;
            ret_op->offset_y = 0;
        }


        return ret_op;
    }
    else
    {
        return NULL;
    }
}


rt_err_t drv_epic_commit_op(drv_epic_operation *op)
{
    priv_render_list_t *rl = drv_epic.using_rl;

    RT_ASSERT(rl);
    RT_ASSERT(0 == (rl->flag & (rl_flag_rendering)));

    HAL_EPIC_AreaMove(&op->clip_area, op->offset_x, op->offset_y);



    if (0xFFFFFFFF != drv_epic.dbg_flag_dis_merge_operations)
    {
        if (rl->src_list_len > 0)
        {
            drv_epic_operation *prev_op = &rl->src_list[rl->src_list_len - 1];
            drv_epic_operation *curr_op = op;

            if ((DRV_EPIC_COLOR_FILL == prev_op->op) && (prev_op->desc.fill.opa >= OPA_MAX) && (NULL == prev_op->mask.data)
                    && (DRV_EPIC_COLOR_BLEND == op->op) && (0 == op->desc.blend.use_dest_as_bg)
                    && (0 == op->desc.blend.layer.transform_cfg.angle)
                    && (EPIC_INPUT_SCALE_NONE == op->desc.blend.layer.transform_cfg.scale_x)
                    && (EPIC_INPUT_SCALE_NONE == op->desc.blend.layer.transform_cfg.scale_y)
               )
            {
                if (drv_epic.dbg_flag_dis_merge_operations & 1) goto __COMMIT_OPERATION;

                EPIC_AreaTypeDef prev_area = prev_op->clip_area;
                //HAL_EPIC_AreaMove(&prev_area, prev_op->offset_x, prev_op->offset_y);
                //HAL_EPIC_AreaMove(&prev_area, -op->offset_x, -op->offset_y);

                /*    if (HAL_EPIC_AreaIsIn(&prev_area, &op->clip_area))
                    {
                        //Merge filling&blending operations to one
                        op->desc.blend.use_dest_as_bg = 0;
                        op->desc.blend.r = prev_op->desc.fill.r;
                        op->desc.blend.g = prev_op->desc.fill.g;
                        op->desc.blend.b = prev_op->desc.fill.b;

                        memcpy(prev_op, op, sizeof(drv_epic_operation));
                        rl->src_list_len--;
                    }
                    else
                */
                if (HAL_EPIC_AreaIsIn(&op->clip_area, &prev_area))
                {
                    //Merge filling&blending operations to one
                    op->desc.blend.use_dest_as_bg = 0;
                    op->desc.blend.r = prev_op->desc.fill.r;
                    op->desc.blend.g = prev_op->desc.fill.g;
                    op->desc.blend.b = prev_op->desc.fill.b;

                    op->clip_area = prev_area;

                    memcpy(prev_op, op, sizeof(drv_epic_operation));
                    rl->src_list_len--;
                }
            }
            else if ((DRV_EPIC_COLOR_FILL == prev_op->op) && (DRV_EPIC_COLOR_FILL == curr_op->op))
            {
                if (drv_epic.dbg_flag_dis_merge_operations & 2) goto __COMMIT_OPERATION;

                if ((NULL == prev_op->mask.data) && (NULL == curr_op->mask.data))
                {
                    EPIC_AreaTypeDef res_area;

                    if (AreaExcept(&res_area, &prev_op->clip_area, &curr_op->clip_area))
                    {
                        RT_ASSERT(HAL_EPIC_AreaIsValid(&res_area));
                        if (drv_epic.dbg_flag_dis_merge_operations & 4) goto __COMMIT_OPERATION;

                        if (curr_op->desc.fill.opa >= OPA_MAX)
                        {
                            memcpy(prev_op, curr_op, sizeof(drv_epic_operation)); //Overwrite previous
                            rl->src_list_len--;
                        }
                        else if (prev_op->desc.fill.opa >= OPA_MAX)
                        {
                            //TODO  Reduce current area, and mix current color to previous color
                        }
                    }
                    else if (AreaExcept(&res_area, &curr_op->clip_area, &prev_op->clip_area))
                    {
                        RT_ASSERT(HAL_EPIC_AreaIsValid(&res_area));
                        if (drv_epic.dbg_flag_dis_merge_operations & 8) goto __COMMIT_OPERATION;

                        if (curr_op->desc.fill.opa >= OPA_MAX)
                        {
                            prev_op->clip_area = res_area; //Reduce previous area
                        }
                        else if (prev_op->desc.fill.opa >= OPA_MAX)
                        {
                            //TODO  Reduce current area, and mix current color to previous color
                        }
                    }
                    else if (HAL_EPIC_AreaIsIn(&prev_op->clip_area, &curr_op->clip_area))
                    {
                        if (drv_epic.dbg_flag_dis_merge_operations & 0x10) goto __COMMIT_OPERATION;

                        if (curr_op->desc.fill.opa >= OPA_MAX)
                        {
                            //print_operation("prev_op",prev_op);
                            //print_operation("curr_op",curr_op);
                            prev_op->desc.fill.r = curr_op->desc.fill.r;
                            prev_op->desc.fill.g = curr_op->desc.fill.g;
                            prev_op->desc.fill.b = curr_op->desc.fill.b;
                            prev_op->desc.fill.opa = curr_op->desc.fill.opa;

                            prev_op->clip_area = curr_op->clip_area;//Overwrite

                            rl->src_list_len--;
                        }
                        else if (HAL_EPIC_AreaIsIn(&curr_op->clip_area, &prev_op->clip_area)) //Same area
                        {
                            //Todo Mix current color to previous color
                        }
                    }

                }
            }

        }
    }

__COMMIT_OPERATION:
    /*Commit operation*/
    RT_ASSERT(rl->src_list_len < src_list_max);
    rl->src_list_len++;
    return RT_EOK;
}


drv_epic_letter_type_t *drv_epic_op_alloc_letter(drv_epic_operation *op)
{
    priv_render_list_t *rl = drv_epic.using_rl;

    RT_ASSERT(rl);
    RT_ASSERT(0 == (rl->flag & (rl_flag_rendering)));
    RT_ASSERT(rl->letter_pool_free < letter_pool_max);

    drv_epic_letter_type_t *p_free = &rl->letter_pool[rl->letter_pool_free];
    rl->letter_pool_free++;


    if (!op->desc.label.p_letters)
    {
        op->desc.label.letter_num = 1;
        op->desc.label.p_letters = p_free;
    }
    else
    {
        RT_ASSERT(op->desc.label.p_letters + op->desc.label.letter_num == p_free);

        op->desc.label.letter_num++;
    }

    return p_free;
}




rt_err_t drv_epic_render_draw_commit(drv_epic_render_draw_cfg *cfg)
{
    bool send_msg;
    priv_render_list_t *rl = (priv_render_list_t *)cfg->render_list;

    rt_enter_critical();
    if (rl_flag_commit & rl->flag)
    {
        send_msg = false;
        //RT_ASSERT(drv_epic.mq->entry > 0);
    }
    else
        send_msg = true;
    rl->flag |= rl_flag_commit;
    rl->flag &= ~rl_flag_overwritting;
    rt_exit_critical();

    HAL_EPIC_AreaCopy(&rl->commit_area, &cfg->area);


    if (send_msg)
    {
        rt_err_t err;

        EPIC_MsgTypeDef msg;

        msg.id = EPIC_MSG_RENDER_DRAW;
        msg.tick = rt_tick_get();
        memcpy(&msg.content.rd, cfg, sizeof(drv_epic_render_draw_cfg));

        err = rt_mq_send(drv_epic.mq, &msg, sizeof(msg));
        if (err != RT_EOK)
        {
            RT_ASSERT(0);
        }
        return err;
    }

    return RT_EOK;
}
#endif

int drv_epic_init(void)
{
    rt_err_t err;

    if (0 == drv_epic_inited)
    {
        memset(&drv_epic, 0, sizeof(EPIC_DrvTypeDef));


#ifdef HAL_EZIP_MODULE_ENABLED
        drv_epic.ezip_handle.Instance = EZIP;
#endif
        drv_epic.split_rd.op = DRV_EPIC_INVALID;


        err = rt_sem_init(&epic_sema, "epic", 1, RT_IPC_FLAG_FIFO);
        RT_ASSERT(RT_EOK == err);

#ifdef DRV_EPIC_NEW_API

        rt_sem_init(&drv_epic.lcd_sema, "rd_lcd", 1, RT_IPC_FLAG_FIFO);


        drv_epic.mq = rt_mq_create("drv_epic", sizeof(EPIC_MsgTypeDef), 4, RT_IPC_FLAG_FIFO);
        RT_ASSERT(drv_epic.mq);

        rt_err_t ret = rt_thread_init(&drv_epic.task, "epic_task", epic_task, &drv_epic, drv_epic_stack, sizeof(drv_epic_stack),
                                      RT_THREAD_PRIORITY_HIGH + RT_THREAD_PRIORITY_LOWWER, RT_THREAD_TICK_DEFAULT);

        RT_ASSERT(RT_EOK == ret);
        rt_thread_startup(&drv_epic.task);

#endif /* DRV_EPIC_NEW_API */

    }
    drv_epic_inited = 1;
    return 0;

}
INIT_DEVICE_EXPORT(drv_epic_init);

#if defined(FINSH_USING_MSH)&&!defined(PY_GEN)
#include <finsh.h>
static rt_err_t drv_epic_cfg(int argc, char **argv)
{
    if (argc < 2)
    {
        LOG_I("drv_epic_cfg [OPTION] [VALUE]");
        LOG_I("    OPTION: break, ramIns, MergeOp");
        return RT_EOK;
    }

    if (strcmp(argv[1], "break") == 0)
    {
        if (argc > 2) //write
        {
            drv_epic.dbg_src_addr = strtoul(argv[2], 0, 16);
        }

        LOG_E("break at src %x.", drv_epic.dbg_src_addr);
    }
    else if (strcmp(argv[1], "ramIns") == 0)
    {
        if (argc > 2) //write
        {
            drv_epic.dbg_flag_dis_ram_instance = strtoul(argv[2], 0, 10);
        }

        LOG_E("Disable Ram Instance = %d.", drv_epic.dbg_flag_dis_ram_instance);
    }
    else if (strcmp(argv[1], "printRl") == 0)
    {
        if (argc > 2) //write
        {
            drv_epic.dbg_flag_print_rl = strtoul(argv[2], 0, 10);
        }

        LOG_E("dbg_flag_print_rl = %d.", drv_epic.dbg_flag_print_rl);
    }
    else if (strcmp(argv[1], "MergeOp") == 0)
    {
        if (argc > 2) //write
        {
            drv_epic.dbg_flag_dis_merge_operations = strtoul(argv[2], 0, 16);
        }

        LOG_E("Disable merge operations = %d.", drv_epic.dbg_flag_dis_merge_operations);
    }

    return RT_EOK;
}
MSH_CMD_EXPORT(drv_epic_cfg,  drv_epic configuration);

#endif

#endif /* BSP_USING_EPIC */
