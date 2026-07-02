/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "drv_epic.h"
#if defined(BSP_USING_EPIC) && !defined(DRV_EPIC_NEW_API)
#include "string.h"
#include "drv_epic_private.h"
#ifdef HAL_EZIP_MODULE_ENABLED
    #include "drv_flash.h"
#endif
extern EPIC_DrvTypeDef drv_epic;
struct rt_semaphore epic_sema;

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



static const char *op_name(drv_epic_op_type_t op)
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

static void gpu_lock(drv_epic_op_type_t ops,
                     void *p1,
                     void *p2,
                     void *p3)
{
#define CALC_LAYER_SIZE(layer) \
        (RT_ALIGN((layer)->total_width * (layer)->height * HAL_EPIC_GetColorDepth((layer)->color_mode), 8) >> 3)
#define GET_LAYER_SIZE(layer) \
        (EPIC_IS_EZIP_COLOR_MODE((layer)->color_mode))?((layer)->data_size):CALC_LAYER_SIZE(layer)

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

        bg_size = GET_LAYER_SIZE(bg);
        fg_size = GET_LAYER_SIZE(fg);

        drv_epic.gpu_output_addr = (uint32_t) dst->data;
        drv_epic.gpu_output_size = CALC_LAYER_SIZE(dst);
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
            mask_size = GET_LAYER_SIZE(mask);
        }
        drv_epic.gpu_fg_addr = (uint32_t) fg->data;
        drv_epic.gpu_bg_addr = (uint32_t) bg->data;

        bg_size = GET_LAYER_SIZE(bg);
        fg_size = GET_LAYER_SIZE(fg);

        drv_epic.gpu_output_addr = (uint32_t) dst->data;
        drv_epic.gpu_output_size = CALC_LAYER_SIZE(dst);

    }
    break;

    case DRV_EPIC_COLOR_FILL:
    {

        EPIC_LayerConfigTypeDef *mask = p1;

        EPIC_LayerConfigTypeDef *param = p3;
        RT_ASSERT(NULL != param);

        drv_epic.gpu_output_addr = (uint32_t) param->data;
        drv_epic.gpu_output_size = CALC_LAYER_SIZE(param);

        if (mask)
        {
            drv_epic.gpu_mask_addr = (uint32_t) mask->data;
            mask_size = GET_LAYER_SIZE(mask);
        }


    }
    break;

    case DRV_EPIC_IMG_COPY:
    {

        EPIC_BlendingDataType *fg = p1;

        EPIC_BlendingDataType *dst = p3;

        drv_epic.gpu_fg_addr = (uint32_t) fg->data;
        drv_epic.gpu_bg_addr = drv_epic.gpu_fg_addr;
        fg_size = GET_LAYER_SIZE(fg);
        bg_size = fg_size;

        drv_epic.gpu_output_addr = (uint32_t) dst->data;
        drv_epic.gpu_output_size = CALC_LAYER_SIZE(dst);
    }
    break;

    case DRV_EPIC_FILL_GRAD:
    {
        EPIC_GradCfgTypeDef *param = p3;

        drv_epic.gpu_output_addr = (uint32_t) param->start;
        drv_epic.gpu_output_size = CALC_LAYER_SIZE(param);
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
    rt_pm_hw_device_start();
#endif /*BSP_USING_PM*/

}

static void gpu_unlock(void)
{
#ifdef BSP_USING_PM
    rt_pm_hw_device_stop();
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

    err = drv_gpu_release();
    RT_ASSERT(RT_EOK == err);



    if (cb) cb(epic);
}

static void epic_abort_callback(EPIC_HandleTypeDef *epic)
{
    drv_epic.split_rd.op = DRV_EPIC_INVALID;
    epic_cplt_callback(epic);
}


/* Bilinear interpolation of a single color channel at (x,y) within area */
static uint8_t grad_interp_ch(uint8_t c00, uint8_t c01, uint8_t c10, uint8_t c11,
                              int16_t x, int16_t y, const EPIC_AreaTypeDef *area)
{
    int32_t w = HAL_EPIC_AreaWidth(area);
    int32_t h = HAL_EPIC_AreaHeight(area);
    int32_t x_num = x - area->x0;
    int32_t y_num = y - area->y0;

    int32_t top = (c00 * (w - x_num) + c01 * x_num) / w;
    int32_t bot = (c10 * (w - x_num) + c11 * x_num) / w;

    return (uint8_t)((top * (h - y_num) + bot * y_num) / h);
}

/* Recalculate a corner color at (x,y) based on full-area gradient corners */
static EPIC_ColorDef grad_interp_color(int16_t x, int16_t y, const EPIC_AreaTypeDef *area)
{
    EPIC_ColorDef c;
    c.ch.alpha = grad_interp_ch(drv_epic.grad_color[0][0].ch.alpha,
                                drv_epic.grad_color[0][1].ch.alpha,
                                drv_epic.grad_color[1][0].ch.alpha,
                                drv_epic.grad_color[1][1].ch.alpha, x, y, area);
    c.ch.color_r = grad_interp_ch(drv_epic.grad_color[0][0].ch.color_r,
                                  drv_epic.grad_color[0][1].ch.color_r,
                                  drv_epic.grad_color[1][0].ch.color_r,
                                  drv_epic.grad_color[1][1].ch.color_r, x, y, area);
    c.ch.color_g = grad_interp_ch(drv_epic.grad_color[0][0].ch.color_g,
                                  drv_epic.grad_color[0][1].ch.color_g,
                                  drv_epic.grad_color[1][0].ch.color_g,
                                  drv_epic.grad_color[1][1].ch.color_g, x, y, area);
    c.ch.color_b = grad_interp_ch(drv_epic.grad_color[0][0].ch.color_b,
                                  drv_epic.grad_color[0][1].ch.color_b,
                                  drv_epic.grad_color[1][0].ch.color_b,
                                  drv_epic.grad_color[1][1].ch.color_b, x, y, area);
    return c;
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
    EPIC_HandleTypeDef *h_epic = &drv_epic.epic_handle;
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

        ret = HAL_EPIC_Adv(h_epic, drv_epic.input_layers,
                           drv_epic.input_layer_cnt, &drv_epic.output_layer);

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

        /* Recalculate corner colors for current tile position */
        param.color[0][0] = grad_interp_color(cur_render_area.x0, cur_render_area.y0, p_render_area);
        param.color[0][1] = grad_interp_color(cur_render_area.x1, cur_render_area.y0, p_render_area);
        param.color[1][0] = grad_interp_color(cur_render_area.x0, cur_render_area.y1, p_render_area);
        param.color[1][1] = grad_interp_color(cur_render_area.x1, cur_render_area.y1, p_render_area);

        LOG_D("EPIC Fill Grad buf=0x%x, cf=%d, total_w=%d w,h=%d,%d",
              param.start, param.color_mode, param.total_width,
              param.width, param.height);

        h_epic->XferCpltCallback = epic_cplt_callback;
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
        EPIC_HandleTypeDef *h_epic = &drv_epic.epic_handle;

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


__ROM_USED EPIC_HandleTypeDef *drv_get_epic_handle(void)
{
    return &drv_epic.epic_handle;
}

#ifdef HAL_EZIP_MODULE_ENABLED
__ROM_USED EZIP_HandleTypeDef *drv_get_ezip_handle(void)
{
    return &drv_epic.ezip_handle;
}
#endif /* HAL_EZIP_MODULE_ENABLED */


#ifdef HAL_JPEGD_MODULE_ENABLED
__ROM_USED JPEGD_HandleTypeDef *drv_get_jpegd_handle(void)
{
    return &drv_epic.jpegd_handle;
}
#endif /* HAL_JPEGD_MODULE_ENABLED */

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
#endif /* HAL_EZIP_MODULE_ENABLED */


#ifdef HAL_JPEGD_MODULE_ENABLED
__ROM_USED void JPEGD_IRQHandler(void)
{
    JPEGD_HandleTypeDef *jpegd;

    rt_interrupt_enter();

    jpegd = drv_get_jpegd_handle();
    RT_ASSERT(RT_NULL != jpegd);

    HAL_JPEGD_IRQHandler(jpegd);

    rt_interrupt_leave();
}
#endif /* HAL_JPEGD_MODULE_ENABLED */



#ifndef __DEBUG__
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
#endif /* __DEBUG__ */

void drv_epic_single_init(void)
{
    rt_err_t err = rt_sem_init(&epic_sema, "epic", 1, RT_IPC_FLAG_FIFO);
    RT_ASSERT(RT_EOK == err);
    drv_epic.split_rd.op = DRV_EPIC_INVALID;
}

void drv_epic_single_open(EPIC_DrvTypeDef *p_drv_epic)
{
    HAL_NVIC_SetPriority(EPIC_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EPIC_IRQn);

#ifdef HAL_EZIP_MODULE_ENABLED
    HAL_NVIC_SetPriority(EZIP_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EZIP_IRQn);
#endif /* HAL_EZIP_MODULE_ENABLED */

#ifdef HAL_JPEGD_MODULE_ENABLED
    HAL_NVIC_SetPriority(JPEGD_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(JPEGD_IRQn);
#endif /* HAL_JPEGD_MODULE_ENABLED */
}



static rt_err_t wait_gpu_done(rt_int32_t time)
{
    rt_err_t err;
    cont_blend_reset();
    do
    {
        err = drv_gpu_take(time);

        if (RT_EOK != err)
        {
            LOG_E("wait_gpu_done timeout(-2)? err=%d", err);
#ifndef __DEBUG__
            gpu_reset();
            epic_abort_callback(&drv_epic.epic_handle);
#else
            RT_ASSERT(0); //Raise an assertion in debug mode.
#endif /* __RELEASE__ */
        }
    }
    while (RT_EOK != err);

    return err;
}


rt_err_t drv_gpu_take(rt_int32_t ms)
{
    rt_err_t err;

    err = rt_sem_take(&epic_sema, rt_tick_from_millisecond(ms));

    if (RT_EOK != err)
    {
        print_gpu_error_info();
    }

    return err;
}

rt_err_t drv_gpu_release(void)
{
    RT_ASSERT(0 == epic_sema.value);
    return rt_sem_release(&epic_sema);
}



rt_err_t drv_gpu_check_done(rt_int32_t ms)
{
    rt_err_t err;

    if (epic_sema.value == 0) // Speed up return result if GPU is NOT working
    {
        err = wait_gpu_done(ms);
        if (RT_EOK == err)
        {
            drv_gpu_release();
        }
    }
    else
    {
        err = RT_EOK;
    }


    return err;
}


bool drv_gpu_is_busy(void)
{
    if (-RT_ETIMEOUT == rt_sem_trytake(&epic_sema))
        return true;
    else
    {
        RT_ASSERT(0 == epic_sema.value);
        rt_sem_release(&epic_sema);
        return false;
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
    EPIC_HandleTypeDef *h_epic = &drv_epic.epic_handle;
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
    EPIC_HandleTypeDef *h_epic = &drv_epic.epic_handle;
    EPIC_AreaTypeDef *fill_area = &drv_epic.split_rd.dst_area;

    err = wait_gpu_done(GPU_BLEND_EXP_MS);
    if (RT_EOK != err) return err;

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
                       uint32_t argb8888,
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


    LOG_D("\r\n drv_epic_fill dst=%p "AreaString" color=%x", dst, AreaParams(dst_area), argb8888);
    if (mask_map)
    {
        LOG_D("mask=%p "AreaString, mask_map, AreaParams(mask_area));
    }

    EPIC_LayerConfigTypeDef *p_output_canvas = &drv_epic.output_layer;
    uint8_t opa;


    HAL_EPIC_LayerConfigInit(p_output_canvas);
    p_output_canvas->color_mode = dst_cf;
    p_output_canvas->total_width = HAL_EPIC_AreaWidth(dst_area);
    opa = (uint8_t)((argb8888 >> 24) & 0xFF);
    p_output_canvas->color_r = (uint8_t)((argb8888 >> 16) & 0xFF);
    p_output_canvas->color_g = (uint8_t)((argb8888 >> 8) & 0xFF);
    p_output_canvas->color_b = (uint8_t)((argb8888 >> 0) & 0xFF);
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
        p_input_layers[1].data = (uint8_t *) mono_layer_addr;
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
#ifndef SF32LB55X
    else if (opa != 255)
    {
        EPIC_LayerConfigTypeDef *p_input_layers = &drv_epic.input_layers[0];

        p_input_layers[1] = *p_output_canvas;
        p_input_layers[1].data = (uint8_t *) mono_layer_addr;
        p_input_layers[1].color_mode = EPIC_INPUT_MONO;
        p_input_layers[1].alpha = opa;
        p_input_layers[1].ax_mode = ALPHA_BLEND_RGBCOLOR;


        p_input_layers[0] = *p_output_canvas;
        p_input_layers[0].color_en = false;

        p_output_canvas->color_en = false;

        err = drv_epic_fill_ext(p_input_layers, 2, p_output_canvas, cbk);
    }
#else /* !SF32LB55X */
    else if (opa != 255)
    {
        EPIC_LayerConfigTypeDef *p_input_layer = &drv_epic.input_layers[0];

        *p_input_layer = *p_output_canvas;
        p_input_layer->alpha = (0 == opa) ? 255 : (256 - opa);
        p_input_layer->color_en = false;

        err = drv_epic_fill_ext(p_input_layer, 1, p_output_canvas, cbk);
    }
#endif /*SF32LB55X */
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
    EPIC_HandleTypeDef *h_epic = &drv_epic.epic_handle;
    EPIC_AreaTypeDef *fill_area = &drv_epic.split_rd.dst_area;

    err = wait_gpu_done(GPU_BLEND_EXP_MS);
    if (RT_EOK != err) return err;

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
    EPIC_HandleTypeDef *h_epic = &drv_epic.epic_handle;
    EPIC_AreaTypeDef *p_blend_area = &drv_epic.split_rd.dst_area;

    err = wait_gpu_done(GPU_BLEND_EXP_MS);
    if (RT_EOK != err) return err;

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
                            drv_epic_cplt_cbk cbk)
{

    RT_ASSERT((RT_NULL != output_canvas) && (RT_NULL != input_layers));

    HAL_StatusTypeDef ret;
    rt_err_t err;
    EPIC_HandleTypeDef *h_epic = &drv_epic.epic_handle;
    EPIC_AreaTypeDef *p_blend_area = &drv_epic.split_rd.dst_area;

    err = wait_gpu_done(GPU_BLEND_EXP_MS);
    if (RT_EOK != err) return err;

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

        ret = HAL_EPIC_Adv(h_epic, input_layers, input_layer_cnt, output_canvas);

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
    EPIC_HandleTypeDef *h_epic = &drv_epic.epic_handle;
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


#endif
