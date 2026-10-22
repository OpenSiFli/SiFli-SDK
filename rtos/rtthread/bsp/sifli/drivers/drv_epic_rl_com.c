/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "drv_epic.h"
#if defined(DRV_EPIC_NEW_API) && defined(BSP_USING_EPIC)
#include "drv_epic_private.h"

char *operation_name(drv_epic_op_type_t op)
{
#define OP_TO_NAME_CASE(op) case op: return #op
    switch (op)
    {
        OP_TO_NAME_CASE(DRV_EPIC_DRAW_IMAGE);
        OP_TO_NAME_CASE(DRV_EPIC_DRAW_FILL);
        OP_TO_NAME_CASE(DRV_EPIC_DRAW_LETTERS);
        OP_TO_NAME_CASE(DRV_EPIC_DRAW_ARC);
        OP_TO_NAME_CASE(DRV_EPIC_DRAW_RECT);
        OP_TO_NAME_CASE(DRV_EPIC_DRAW_LINE);
        OP_TO_NAME_CASE(DRV_EPIC_DRAW_BORDER);
        OP_TO_NAME_CASE(DRV_EPIC_DRAW_POLYGON);
        OP_TO_NAME_CASE(DRV_EPIC_DRAW_SHADOW);

    default:
        return "UNKNOW";
    }
}


void print_operation(const char *name, const drv_epic_operation *op)
{


    LOG_E("<<%s, %s>>", name, operation_name(op->op));
    LOG_E("Clip area "AreaString, AreaParams(&op->clip_area));
    if (op->mask.data)
    {
        LOG_E(FORMATED_LAYER_INFO(&op->mask, "Mask"));
    }

    switch (op->op)
    {
    case DRV_EPIC_DRAW_IMAGE:
    {
        LOG_E(FORMATED_LAYER_INFO(&op->desc.blend.layer, "FG"));
        LOG_E(FORMATED_LAYER_EXTRA_INFO(&op->desc.blend.layer));
        if (EPIC_BLEND_MODE_FIXED_BG == op->desc.blend.use_dest_as_bg)
        {
            LOG_E("BG rgb:%d,%d,%d",
                  op->desc.blend.r,
                  op->desc.blend.g,
                  op->desc.blend.b);
        }
    }
    break;

    case DRV_EPIC_DRAW_FILL:
    {
        LOG_E("Fill argb:%d,%d,%d,%d",
              op->desc.fill.opa,
              op->desc.fill.r,
              op->desc.fill.g,
              op->desc.fill.b);
    }
    break;


    case DRV_EPIC_DRAW_LETTERS:
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

    case DRV_EPIC_DRAW_ARC:
    {
        LOG_E("center[%d,%d], angle[%d~%d], round=%d,%d, w=%d, r=%d, argb:0x%08x",
              op->desc.arc.center_x, op->desc.arc.center_y,
              op->desc.arc.start_angle, op->desc.arc.end_angle,
              op->desc.arc.round_start, op->desc.arc.round_end,
              op->desc.arc.width, op->desc.arc.radius,
              op->desc.arc.argb8888
             );
    }
    break;

    case DRV_EPIC_DRAW_RECT:
    {
        LOG_E(AreaString" r=%d, top/bot_fillet=%d,%d argb:0x%08x",
              AreaParams(&op->desc.rectangle.area), op->desc.rectangle.radius,

              op->desc.rectangle.top_fillet, op->desc.rectangle.bot_fillet,

              op->desc.rectangle.argb8888
             );
    }
    break;

    case DRV_EPIC_DRAW_LINE:
    {
        LOG_E("p1=%d,%d, p2=%d,%d, w=%d, round=%d,%d argb:0x%08x",
              op->desc.line.p1.x, op->desc.line.p1.y,
              op->desc.line.p2.x, op->desc.line.p2.y,
              op->desc.line.width,
              op->desc.line.round_start, op->desc.line.round_end,
              op->desc.line.argb8888
             );
    }
    break;

    case DRV_EPIC_DRAW_BORDER:
    {
        LOG_E(AreaString" r=%d, w=%d, argb:0x%08x, tblr=%d%d%d%d",
              AreaParams(&op->desc.border.area), op->desc.border.radius,
              op->desc.border.width, op->desc.border.argb8888,
              op->desc.border.top_side, op->desc.border.bot_side,
              op->desc.border.left_side, op->desc.border.right_side
             );
    }
    break;

    case DRV_EPIC_DRAW_SHADOW:
        LOG_E("Shadow area="AreaString", radius=%d width=%d spread=%d ofs=%d,%d",
              AreaParams(&op->desc.shadow.area), op->desc.shadow.radius,
              op->desc.shadow.width, op->desc.shadow.spread,
              op->desc.shadow.ofs_x, op->desc.shadow.ofs_y);
        break;

    case DRV_EPIC_DRAW_POLYGON:
    {
        LOG_E("argb:0x%08x, cnt:%d ", op->desc.polygon.argb8888, op->desc.polygon.point_cnt);
    }
    break;


    default:
    {
        LOG_E("Unknow %d", op->op);
    }
    break;
    }
}


uint32_t GetElapsedUs(uint32_t prev_tick, uint32_t cur_tick)
{
    if (0 == gp_drv_epic->hclk_freq_Mhz)
    {
        gp_drv_epic->hclk_freq_Mhz = HAL_RCC_GetHCLKFreq(CORE_ID_CURRENT) / 1000000;
    }

    return (HAL_GetElapsedTick(prev_tick, cur_tick) + (gp_drv_epic->hclk_freq_Mhz >> 1)) / gp_drv_epic->hclk_freq_Mhz;
}

__ROM_USED EPIC_HandleTypeDef *drv_get_epic_handle(void)
{
    return &gp_drv_epic->epic_handle;
}

#ifdef HAL_EZIP_MODULE_ENABLED
__ROM_USED EZIP_HandleTypeDef *drv_get_ezip_handle(void)
{
    return &gp_drv_epic->ezip_handle;
}
#endif /* HAL_EZIP_MODULE_ENABLED */


#ifdef HAL_JPEGD_MODULE_ENABLED
__ROM_USED JPEGD_HandleTypeDef *drv_get_jpegd_handle(void)
{
    return &gp_drv_epic->jpegd_handle;
}
#endif /* HAL_JPEGD_MODULE_ENABLED */

#endif /*DRV_EPIC_NEW_API && BSP_USING_EPIC*/
