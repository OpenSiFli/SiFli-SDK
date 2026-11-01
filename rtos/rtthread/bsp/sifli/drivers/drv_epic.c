/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "drv_epic.h"
#ifdef BSP_USING_EPIC
#include <rthw.h>
#include "string.h"
#include "mem_section.h"
#ifdef HAL_EZIP_MODULE_ENABLED
    #include "drv_flash.h"
#endif

#include "drv_epic_private.h"
#if defined(DRV_EPIC_ARC_MASK_VGLITE) && defined(DRV_EPIC_NEW_API_DUAL_CORE_HCPU)
#include "acpu_ctrl.h"
#endif


#ifdef USING_VGLITE
    #include "drv_vglite.h"
#endif
#if defined(DRV_EPIC_ARC_MASK_VGLITE) && !defined(DRV_EPIC_NEW_API_DUAL_CORE_HCPU)
    #include "drv_epic_arc_vglite.h"
#endif



static EPIC_HandleTypeDef *epic = NULL;



static void dummy_function_for_source_insight_symbol_list0(void)
{
}

L1_RET_BSS_SECT_BEGIN(drv_epic_ram)
EPIC_DrvTypeDef drv_epic;
EPIC_DrvTypeDef *gp_drv_epic = NULL;
L1_RET_BSS_SECT_END




static void dummy_function_for_source_insight_symbol_list1(void)
{
}


int drv_epic_init(void);












void drv_gpu_open(void)
{
#ifdef HAL_EPIC_MODULE_ENABLED
#ifdef HAL_JPEGD_MODULE_ENABLED
    uint32_t max_img_width;
#endif /* HAL_JPEGD_MODULE_ENABLED */

    if (NULL == epic)
    {
        if (NULL == gp_drv_epic) drv_epic_init();
        epic = &drv_epic.epic_handle;

        epic->Instance = EPIC;
        epic->RamInstance = &drv_epic.RamEPIC;

#ifdef EPIC_DEBUG
        epic->op_hist = &drv_epic.epic_op_hist;
#endif /* EPIC_DEBUG */

#ifdef HAL_EZIP_MODULE_ENABLED
        epic->hezip = &drv_epic.ezip_handle;
        epic->hezip->Instance = EZIP;
        /*
            There are some reasons to set flash_handle_query_cb to NULL:
            1. It cannot skip bad blocks on NAND flash.
            2. Direct reading may cause NAND flash read-back error.
            3. It may cause bus error on dual-core mode if the cb function located on some memory which is not accessible by another core, for example, ITCM.
        */
        epic->hezip->flash_handle_query_cb = NULL;
        epic->hezip->RamInstance = &drv_epic.RamEZIP;

        HAL_EZIP_Init(epic->hezip);
#endif /* HAL_EZIP_MODULE_ENABLED */

#ifdef HAL_JPEGD_MODULE_ENABLED
        epic->hjpegd = &drv_epic.jpegd_handle;
        epic->hjpegd->Instance = hwp_jpegd;
        epic->hjpegd->RamInstance = &drv_epic.RamJPEGD;
        HAL_JPEGD_Init(epic->hjpegd, NULL, NULL);
#ifdef LV_HOR_RES_MAX
        max_img_width = ((LV_HOR_RES_MAX + 15 + 15) >> 4) << 4;
#else
        max_img_width = 640;
#endif /* LV_HOR_RES_MAX */

        epic->jpegd_work_buf_size = max_img_width * 2 * 32; /* 2bytes per pixel, 32 rows */
        RT_ASSERT(NULL == epic->jpegd_work_buf);
        epic->jpegd_work_buf = epic_malloc(epic->jpegd_work_buf_size);
        RT_ASSERT(epic->jpegd_work_buf);

#endif /* HAL_JPEGD_MODULE_ENABLED */

        HAL_EPIC_Init(epic);


#ifdef DRV_EPIC_NEW_API
        memcpy(&drv_epic.epic_handle2, &drv_epic.epic_handle, sizeof(drv_epic.epic_handle2));
#ifdef HAL_EZIP_MODULE_ENABLED
        memcpy(&drv_epic.ezip_handle2, &drv_epic.ezip_handle, sizeof(drv_epic.ezip_handle2));
        drv_epic.epic_handle2.hezip = &drv_epic.ezip_handle2;
#endif
#ifdef HAL_JPEGD_MODULE_ENABLED
        memcpy(&drv_epic.jpegd_handle2, &drv_epic.jpegd_handle, sizeof(drv_epic.jpegd_handle2));
        drv_epic.epic_handle2.hjpegd = &drv_epic.jpegd_handle2;
#endif
#else
        drv_epic_single_open(&drv_epic);
#endif /* DRV_EPIC_NEW_API */
        LOG_I("drv_gpu opened.");
    }
#endif


#ifdef USING_VGLITE
    drv_vglite_open();
#endif
}


void drv_gpu_close(void)
{
#ifdef HAL_EPIC_MODULE_ENABLED
    if (epic)
    {
        RT_ASSERT(NULL != gp_drv_epic);
        drv_gpu_check_done(GPU_BLEND_EXP_MS);
        RT_ASSERT(epic->State != HAL_EPIC_STATE_BUSY);
#if defined(DRV_EPIC_ARC_MASK_VGLITE) && defined(DRV_EPIC_NEW_API_DUAL_CORE_HCPU)
        epic_rl_arg_t arg = {gp_drv_epic, NULL, NULL};
        rt_err_t ret = rt_sem_take(&gp_drv_epic->render_sema, RT_WAITING_FOREVER);
        RT_ASSERT(ret == RT_EOK);
        ret = (rt_err_t)acpu_run_task(ACPU_TASK_epic_rl, &arg, sizeof(arg), NULL);
        rt_sem_release(&gp_drv_epic->render_sema);
        if (ret != RT_EOK) LOG_E("ACPU VGLite close failed: %d", ret);
#endif
#ifdef HAL_EZIP_MODULE_ENABLED
        if (epic->hezip)
        {
            RT_ASSERT(epic->hezip->State != HAL_EZIP_STATE_BUSY);
            HAL_EZIP_DeInit(epic->hezip);
        }
#endif /* HAL_EZIP_MODULE_ENABLED */

#ifdef HAL_JPEGD_MODULE_ENABLED
        if (epic->hjpegd)
        {
            HAL_JPEGD_DeInit(epic->hjpegd);
        }
        if (epic->jpegd_work_buf)
        {
            epic_free(epic->jpegd_work_buf);
            epic->jpegd_work_buf = NULL;
        }
#endif /* HAL_JPEGD_MODULE_ENABLED */

        epic = NULL;
        LOG_I("drv_gpu closed.");
    }
#endif

#ifdef USING_VGLITE
#if defined(DRV_EPIC_ARC_MASK_VGLITE) && !defined(DRV_EPIC_NEW_API_DUAL_CORE_HCPU)
    if (drv_epic_arc_vglite_idle())
#endif
    drv_vglite_close();
#endif

}

uint32_t drv_epic_get_error(void)
{
    uint32_t ret = 0;
    if (drv_epic.gpu_timeout_cnt)
    {
        ret = drv_epic.gpu_timeout_cnt;
        drv_epic.gpu_timeout_cnt = 0;
        LOG_I("gpu_timeout_cnt cleared.");
    }

    return ret;
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

        case EPIC_OP_CONT_BLENDING:
        {
            LOG_E("EPIC_OP_CONT_BLENDING");

            print_LayerInfo("fg", (const EPIC_BlendingDataType *)&param->cont_blend_param.input_layer);
            print_LayerInfo("bg", (const EPIC_BlendingDataType *)&param->cont_blend_param.mask_layer);
            print_LayerInfo("dst", (const EPIC_BlendingDataType *)&param->cont_blend_param.output_layer);
        }
        break;

        default:
            break;
        }
    }
}
#endif /* EPIC_DEBUG */
#endif /* HAL_EPIC_MODULE_ENABLED */



void print_gpu_error_info(void)
{
    LOG_E("print_gpu_error_info");
    drv_epic.gpu_timeout_cnt++;

#ifdef HAL_EPIC_MODULE_ENABLED
    RT_ASSERT(epic != NULL);

    if ((HAL_EPIC_STATE_READY != epic->State) || (epic->Instance->STATUS & EPIC_STATUS_IA_BUSY))
    {
        LOG_E("Epic not ready %d, HW busy =%d, ErrorCode %d", epic->State, (epic->Instance->STATUS & EPIC_STATUS_IA_BUSY),
              epic->ErrorCode);

        if (!HAL_RCC_IsModuleEnabled(RCC_MOD_EPIC))
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
#ifdef EPIC_COENG_CFG_JPEG_EN
            if (epic->Instance->COENG_CFG & EPIC_COENG_CFG_JPEG_EN)
            {
                LOG_E("%d is JPEG(0:VL 1:L0 2:L1 3:L2)", (epic->Instance->COENG_CFG & EPIC_COENG_CFG_EZIP_CH_SEL_Msk) >> EPIC_COENG_CFG_EZIP_CH_SEL_Pos);
            }
#endif
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

#define EXAMINE_EZIP_ERROR(err) if(hezip->ErrorCode & err) {LOG_E("Ezip error("#err")");}

        EXAMINE_EZIP_ERROR(EZIP_INT_STA_ROW_ERR_STA);
        EXAMINE_EZIP_ERROR(EZIP_INT_STA_BTYPE_ERR_STA);
        EXAMINE_EZIP_ERROR(EZIP_INT_STA_ETYPE_ERR_STA);
#ifdef EZIP_INT_STA_FTYPE_ERR_STA
        EXAMINE_EZIP_ERROR(EZIP_INT_STA_FTYPE_ERR_STA);
        EXAMINE_EZIP_ERROR(EZIP_INT_STA_WIND_ERR_STA);
#endif /* EZIP_INT_STA_FTYPE_ERR_STA */

        if (!HAL_RCC_IsModuleEnabled(RCC_MOD_EZIP))
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

#ifdef HAL_JPEGD_MODULE_ENABLED
    JPEGD_HandleTypeDef *hjpegd = epic->hjpegd;

    RT_ASSERT(hjpegd != NULL);
    if ((HAL_JPEGD_STATE_READY != hjpegd->State) || (hjpegd->Instance->JPEGD_EN & JPEGD_JPEGD_EN_JPEGD_EN))
    {
        LOG_E("JPEGD not ready %d, HW busy=%x, ErrorCode %xh", hjpegd->State, (hjpegd->Instance->JPEGD_EN & JPEGD_JPEGD_EN_JPEGD_EN),
              hjpegd->ErrorCode);
        if (!HAL_RCC_IsModuleEnabled(RCC_MOD_JPEGD))
        {
            LOG_E("JPEGD not open");
        }
        else
        {
            LOG_E("JPEGD src=%x, src_len=%x, area x0y0x1y1[%d,%d,%d,%d]",
                  hjpegd->Instance->SRC_ADDR,                  hjpegd->Instance->SRC_LEN,
                  hjpegd->Instance->START_POINT >> 16, hjpegd->Instance->START_POINT & 0xFFFF,
                  hjpegd->Instance->END_POINT >> 16, hjpegd->Instance->END_POINT & 0xFFFF);

            LOG_E("ouput %d (0: epic, 1: ahb), buf=%x",
                  GET_REG_VAL2(hjpegd->Instance->JPEGD_PARA, JPEGD_JPEGD_PARA_OUT_SEL),
                  hjpegd->Instance->BUF_ADDR);
        }
    }
    else
    {
        LOG_E("JPEGD is ready");
    }
#endif /* HAL_JPEGD_MODULE_ENABLED */

#ifdef EPIC_DEBUG
    print_epic_op_history();
#endif /* EPIC_DEBUG */


#endif /* HAL_EPIC_MODULE_ENABLED */
}







bool drv_epic_is_busy(void)
{
    if (NULL == gp_drv_epic) return false;
    return drv_gpu_is_busy();
}


int drv_epic_init(void)
{
    rt_err_t err;

    if (NULL == gp_drv_epic)
    {
        memset(&drv_epic, 0, sizeof(EPIC_DrvTypeDef));
        gp_drv_epic = &drv_epic;

        /*
            Init semaphore and EZIP handle, because EZIP decompression may be used before gpu_open
        */
#ifdef HAL_EZIP_MODULE_ENABLED
        drv_epic.ezip_handle.Instance = EZIP;
#endif

#ifdef DRV_EPIC_NEW_API
        drv_epic_render_list_init();

        drv_epic.dbg_flag_print_rl = 0;
        drv_epic.dbg_flag_print_exe_detail = 0;
        drv_epic.dbg_flag_print_statistics = 0;
        drv_epic.dbg_flag_dis_ram_instance = 0;

        drv_epic.dbg_mask_buf_pool_max = mask_buf_max_bytes;
        drv_epic.dbg_render_buf_max = UINT32_MAX;
        drv_epic.letter_buf_pool_max = letter_pool_max;
        drv_epic.rotated = DRV_EPIC_ROT_NONE;
#else
        drv_epic_single_init();
#endif /* DRV_EPIC_NEW_API */

    }
    return 0;

}
INIT_PRE_APP_EXPORT(drv_epic_init);

#if defined(FINSH_USING_MSH)&&!defined(PY_GEN)
#include <finsh.h>
static rt_err_t drv_epic_cfg(int argc, char **argv)
{
    if (argc < 2)
    {
        LOG_I("drv_epic_cfg [OPTION] [VALUE]");
        LOG_I("  [OPTION]:");
#ifdef DRV_EPIC_NEW_API
        LOG_I("    break: set the source address for break point.");
        LOG_I("    ramIns: disable the ram instance, 0: enable, 1: disable.");
        LOG_I("    MergeOp: disable the merge operations, 0: enable, 1: disable.");
        LOG_I("    DisOp: disable the operations, 0: enable, 1: disable.");
        LOG_I("    printRl: print the render list, 0: disable, 1: enable.");
        LOG_I("    printDetail: print the detail of rendering, 0: disable, 1: enable.");
        LOG_I("    printSts: print the statistics of rendering, 0: disable, 1: enable.");
        LOG_I("    mask_max: set the max size of mask buffer pool, default is %d.", drv_epic.dbg_mask_buf_pool_max);
        LOG_I("    render_buf_max: set the max size of render buffer, default is %d.", drv_epic.dbg_render_buf_max);
#endif /* DRV_EPIC_NEW_API */
        return RT_EOK;
    }

#ifdef DRV_EPIC_NEW_API
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
    else if (strcmp(argv[1], "printDetail") == 0)
    {
        if (argc > 2) //write
        {
            drv_epic.dbg_flag_print_exe_detail = strtoul(argv[2], 0, 10);
        }

        LOG_E("print_exe_detail = %d.", drv_epic.dbg_flag_print_exe_detail);
    }
    else if (strcmp(argv[1], "printSts") == 0)
    {
        if (argc > 2) //write
        {
            drv_epic.dbg_flag_print_statistics = strtoul(argv[2], 0, 10);
        }

        LOG_E("print_statistics = %d.", drv_epic.dbg_flag_print_statistics);
    }
    else if (strcmp(argv[1], "MergeOp") == 0)
    {
        if (argc > 2) //write
        {
            drv_epic.dbg_flag_dis_merge_operations = strtoul(argv[2], 0, 16);
        }

        LOG_E("Disable merge operations = %d.", drv_epic.dbg_flag_dis_merge_operations);
    }
    else if (strcmp(argv[1], "DisOp") == 0)
    {
        if (argc > 2) //write
        {
            drv_epic.dbg_flag_dis_operations = strtoul(argv[2], 0, 16);
        }

        LOG_E("Disable operations = 0x%x.", drv_epic.dbg_flag_dis_operations);
    }
    else if (strcmp(argv[1], "mask_max") == 0)
    {
        if (argc > 2) //write
        {
            drv_epic.dbg_mask_buf_pool_max = strtoul(argv[2], 0, 10);
        }

        LOG_E("dbg_mask_buf_pool_max = %d.", drv_epic.dbg_mask_buf_pool_max);
    }
    else if (strcmp(argv[1], "render_buf_max") == 0)
    {
        if (argc > 2) //write
        {
            drv_epic.dbg_render_buf_max = strtoul(argv[2], 0, 10);
        }

        LOG_E("dbg_render_buf_max = %d.", drv_epic.dbg_render_buf_max);
    }
#endif /* DRV_EPIC_NEW_API */



    return RT_EOK;
}
MSH_CMD_EXPORT(drv_epic_cfg,  drv_epic configuration);

#endif

#endif /* BSP_USING_EPIC */
