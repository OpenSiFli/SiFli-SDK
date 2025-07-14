/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bf0_hal.h"
#include "string.h"
//#include "utest.h"
//#include "rtthread.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup EPIC
  * @{
  */

#if defined(HAL_EPIC_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)
//extern void rt_kprintf(const char *fmt, ...);

#define EPIC_PRINTF(...)  //rt_kprintf(__VA_ARGS__)

/**  >> EPIC_TRIGO_SHIFT to normalize*/

#define EPIC_MATH_MIN(a, b) ((a) < (b) ? (a) : (b))
#define EPIC_MATH_MIN3(a, b, c) (EPIC_MATH_MIN(EPIC_MATH_MIN(a,b), c))
#define EPIC_MATH_MIN4(a, b, c, d) (EPIC_MATH_MIN(EPIC_MATH_MIN(a,b), EPIC_MATH_MIN(c,d)))

#define EPIC_MATH_MAX(a, b) ((a) > (b) ? (a) : (b))
#define EPIC_MATH_MAX3(a, b, c) (EPIC_MATH_MAX(EPIC_MATH_MAX(a,b), c))
#define EPIC_MATH_MAX4(a, b, c, d) (EPIC_MATH_MAX(EPIC_MATH_MAX(a,b), EPIC_MATH_MAX(c,d)))

#define EPIC_MATH_ABS(x) ((x) > 0 ? (x) : (-(x)))

#define EPIC_LAYER_MAX_COORDINATE  (EPIC_L0_TL_POS_X0_Msk >> EPIC_L0_TL_POS_X0_Pos)

#if (EPIC_VL_SCALE_RATIO_XPITCH_MAX!=EPIC_VL_SCALE_RATIO_YPITCH_MAX)
    #error "Fix me."
#else
    #define EPIC_VL_SCALE_RATIO_PITCH_MAX   EPIC_VL_SCALE_RATIO_XPITCH_MAX
#endif

#define EPIC_INPUT_SCALING_FACTOR_1  (EPIC_INPUT_SCALE_NONE)

#ifndef SF32LB55X
    #define EPIC_SCALE_FACTOR_FRAC_SIZE  (16)
#else
    #define EPIC_SCALE_FACTOR_FRAC_SIZE  (8)
#endif /* SF32LB55X */

#if EPIC_INPUT_SCALE_FRAC_SIZE > EPIC_SCALE_FACTOR_FRAC_SIZE
    #define EPIC_CONV_SCALE_FACTOR(x)    (((uint32_t)(x)) >> (EPIC_INPUT_SCALE_FRAC_SIZE - EPIC_SCALE_FACTOR_FRAC_SIZE))  //x unit is 'EPIC_INPUT_SCALING_FACTOR_1'
#else
    #define EPIC_CONV_SCALE_FACTOR(x)    (((uint32_t)(x)) << (EPIC_SCALE_FACTOR_FRAC_SIZE - EPIC_INPUT_SCALE_FRAC_SIZE))  //x unit is 'EPIC_INPUT_SCALING_FACTOR_1'
#endif

#define EPIC_IS_VALID_SCALE(x)       (EPIC_CONV_SCALE_FACTOR(x) <= EPIC_VL_SCALE_RATIO_PITCH_MAX)
#define EPIC_SCALE_1                 (1<<EPIC_SCALE_FACTOR_FRAC_SIZE)


#define EPIC_SCALE_INT16(v, pitch, round_v)                (((((int32_t) (v)) << EPIC_SCALE_FACTOR_FRAC_SIZE) + ((int32_t)(round_v))) / ((int32_t)(pitch)))
#define EPIC_RSCALE_INT16(v, pitch, round_v)                ((int16_t)((((float) (v)) * ((float)(pitch)) + ((float)(round_v))) / ((float)EPIC_SCALE_1)))

#define EPIC_SCALE_INT32(v, pitch, round_v)                ((int32_t)(((((float) (v)) * ((float)EPIC_SCALE_1)) + ((float)(round_v))) / ((float)(pitch))))
#define EPIC_RSCALE_INT32(v, pitch, round_v)                ((int32_t)((((float) (v)) * ((float)(pitch)) + ((float)(round_v))) / ((float)EPIC_SCALE_1)))



#define EPCI_IS_TLBR_OVERFLOW(layer, x0, y0, x1, y1) (0 != (((x0) & (~(layer##_TL_POS_X0_Msk >> layer##_TL_POS_X0_Pos))) \
                                                     |((y0) & (~(layer##_TL_POS_Y0_Msk >> layer##_TL_POS_Y0_Pos))) \
                                                     |((x1) & (~(layer##_BR_POS_X1_Msk >> layer##_BR_POS_X1_Pos))) \
                                                     |((y1) & (~(layer##_BR_POS_Y1_Msk >> layer##_BR_POS_Y1_Pos)))) \
                                                     || ((x0)>(x1) || ((y0)>(y1))))

#define EPCI_IS_TL_OVERFLOW(layer, x0, y0, x1, y1) (0 != (((x0) & (~(layer##_TL_POS_X0_Msk >> layer##_TL_POS_X0_Pos))) \
                                                         |((y0) & (~(layer##_TL_POS_Y0_Msk >> layer##_TL_POS_Y0_Pos)))) \
                                                         || ((x0)>(x1) || ((y0)>(y1))))

#define EPIC_DEBUG_PRINT_LAYER_INFO(layer, layer_name) \
                    EPIC_PRINTF("layer %s,cf=%d,data=%x,total_w=%d,area[x0y0(%d,%d),x1y1(%d,%d)] frac[%x,%x]\n", \
                                (layer_name), (layer)->color_mode, (layer)->data, (layer)->total_width,\
                                (layer)->x_offset, \
                                (layer)->y_offset, \
                                (layer)->x_offset + (layer)->width - 1, \
                                (layer)->y_offset + (layer)->height - 1,\
                                (layer)->x_offset_frac, (layer)->y_offset_frac \
                                );



#define EPIC_DEBUG_PRINT_AREA_INFO(area, area_name) \
                    EPIC_PRINTF("area %s,area[x0y0(%d,%d),x1y1(%d,%d)]\n", \
                                area_name, (area)->x0, (area)->y0, (area)->x1, (area)->y1);

//v and pivot_v must have same origin
#define EPIC_MIRROR_V(v, pivot_v) ((pivot_v) - ((v) - (pivot_v)))

#define IS_NEED_TRANSFROM(rot_cfg) ((EPIC_INPUT_SCALING_FACTOR_1 != (rot_cfg)->scale_x) || (EPIC_INPUT_SCALING_FACTOR_1 != (rot_cfg)->scale_y) || (0 != (rot_cfg)->angle)\
                                    || (0 != (rot_cfg)->h_mirror)|| (0 != (rot_cfg)->v_mirror))
#define IS_Ax_COLOR_MODE(c)        ((EPIC_COLOR_A2 == (c)) || (EPIC_COLOR_A4 == (c)) || (EPIC_COLOR_A8 == (c)) || (EPIC_COLOR_MONO == (c)))
#define IS_YUV_COLOR_MODE(c)       (EPIC_COLOR_YUV_FLAG == (EPIC_COLOR_YUV_FLAG&(c)))
#define IS_NO_ALPHA_COLOR_MODE(c)  ((EPIC_COLOR_RGB565 == (c)) || (EPIC_COLOR_RGB888 == (c)) || IS_YUV_COLOR_MODE(c) || (EPIC_COLOR_MONO == (c)))
#define IS_EZIP_COLOR_MODE(c)       (EPIC_COLOR_EZIP_FLAG == (EPIC_COLOR_EZIP_FLAG&(c)))
#define IS_MASK_MODE(m)            ((ALPHA_BLEND_MASK == (m)) || (ALPHA_BLEND_OVERWRITE == (m)))
#define IS_ALPHA_MASK_LAYER(layer) (IS_Ax_COLOR_MODE((layer)->color_mode) && IS_MASK_MODE((layer)->ax_mode))


#define IS_FRAC_COORD_LAYER(layer)   ((0 != (layer)->x_offset_frac)||(0 != (layer)->y_offset_frac))
#define EPIC_TO_INT32_COORD_INNER(INT16,SCALE_INT)   ((int32_t)  ((((int32_t) (INT16)&0xFFFF) << 16) - ((int32_t)(SCALE_INT))))
#define EPIC_INT32_COORD_GET_SCALE_INT(_INT32_) ((uint16_t)   (0x10000 - ((_INT32_)&0xFFFF)))
#define EPIC_INT32_COORD_GET_OFFSET(_INT32_)      ((int16_t)    (((_INT32_) + 0xFFFF) >> 16))
#define IS_NEED_TRANSFROM_LAYER(layer,rot_cfg) (IS_FRAC_COORD_LAYER(layer)||IS_NEED_TRANSFROM(rot_cfg))

#if defined(SF32LB55X)||defined(SF32LB58X)
    #define EPIC_PREFETCH_THRESHOLD  63   //In bytes
#else
    #define EPIC_PREFETCH_THRESHOLD  0   //In bytes. Auto prefetch after 56x, enable prefetch always.
#endif



#define IS_REAL_INSTANCE(ins) (((uint32_t)(EPIC)) == ((uint32_t)(ins)))

#define RETURN_ERROR(hepic,ret_v) \
        do{ (hepic)->ErrorCode = __LINE__;  return ret_v; }while(0)




typedef enum
{
    EPIC_LAYER_IDX_0 = 0,
    EPIC_LAYER_IDX_1 = 1, //Invalid on 52x.
    EPIC_LAYER_IDX_2 = 2, //Invalid on 52x. Transfrom function only on 58x.
    EPIC_LAYER_IDX_VL = 3,
    EPIC_LAYER_IDX_MAX,   //Normal layer maximum num

    EPIC_SPECIAL_LAYER_MASK,  //Special layer: Mask layer
} EPIC_LAYER_IDX;


enum
{
    CONT_BG_LAYER = 0,
    CONT_FG_LAYER = 1,
    CONT_MASK_LAYER = 2,
};




typedef struct
{
    /*Common*/
    uint16_t width;           /**< original layer width*/
    uint16_t height;          /**< original layer heigth*/
    EPIC_AreaTypeDef clip_area;/**< Relative to original buf's TL*/


    /*Scale*/
    uint32_t epic_scale_x;       //EPIC scale x value
    uint32_t epic_scale_y;       //EPIC scale y value
    uint32_t scale_init_x;
    uint32_t scale_init_y;

    /*Rotate*/
    int16_t angle_degree;
    uint16_t abs_sinma;
    uint16_t abs_cosma;
    EPIC_PointTypeDef src_img;
    EPIC_PointTypeDef pivot;
    uint16_t rotated_width;
    uint16_t rotated_height;

    /*Mirror*/
    int8_t h_mirror;
    int8_t v_mirror;
} EPIC_TransformResultDef;

/** sin table for 0~90 degree, Q1.15 */
static const int16_t sin0_90_table[] =
{
    0,     572,   1144,  1715,  2286,  2856,  3425,  3993,  4560,  5126,  5690,  6252,  6813,  7371,  7927,  8481,
    9032,  9580,  10126, 10668, 11207, 11743, 12275, 12803, 13328, 13848, 14364, 14876, 15383, 15886, 16383, 16876,
    17364, 17846, 18323, 18794, 19260, 19720, 20173, 20621, 21062, 21497, 21925, 22347, 22762, 23170, 23571, 23964,
    24351, 24730, 25101, 25465, 25821, 26169, 26509, 26841, 27165, 27481, 27788, 28087, 28377, 28659, 28932, 29196,
    29451, 29697, 29934, 30162, 30381, 30591, 30791, 30982, 31163, 31335, 31498, 31650, 31794, 31927, 32051, 32165,
    32269, 32364, 32448, 32523, 32587, 32642, 32687, 32722, 32747, 32762, 32767
};


//static struct rt_mutex epic_mutex;
//static bool epic_init_done = false;

static void EPIC_RotationCpltCallback(EPIC_HandleTypeDef *epic);
static void EPIC_BlendCpltCallback(EPIC_HandleTypeDef *epic);
static void EPIC_FillCpltCallback(EPIC_HandleTypeDef *epic);
static void EPIC_CopyCpltCallback(EPIC_HandleTypeDef *epic);
static void EPIC_GradCpltCallback(EPIC_HandleTypeDef *epic);

#ifdef HAL_EZIP_MODULE_ENABLED
    static void EPIC_EzipCpltCallback(EZIP_HandleTypeDef *ezip);
#endif /* HAL_EZIP_MODULE_ENABLED */

static HAL_StatusTypeDef EPIC_ConfigRotation(EPIC_HandleTypeDef *epic, EPIC_TransformCfgTypeDef *rot_cfg,
        EPIC_BlendingDataType *fg, EPIC_BlendingDataType *bg,
        EPIC_BlendingDataType *dst, uint8_t alpha);
static HAL_StatusTypeDef EPIC_ConfigBlend(EPIC_HandleTypeDef *epic, EPIC_BlendingDataType *bg,
        EPIC_BlendingDataType *fg, EPIC_BlendingDataType *dst, uint8_t alpha);

static HAL_StatusTypeDef EPIC_ConfigBlendEx(EPIC_HandleTypeDef *epic,
        EPIC_BlendingDataType *input,
        uint8_t *alpha,
        EPIC_TransformCfgTypeDef *transform_cfg,
        uint8_t layer_num);
static HAL_StatusTypeDef EPIC_ConfigFilling(EPIC_HandleTypeDef *epic, EPIC_FillingCfgTypeDef *param);
static HAL_StatusTypeDef EPIC_ConfigGrad(EPIC_HandleTypeDef *epic, EPIC_GradCfgTypeDef *param);
void EPIC_GetRotatedArea(EPIC_AreaTypeDef *output, uint16_t w, uint16_t h, int16_t angle,
                         const EPIC_PointTypeDef *pivot);

/** Return with sinus of an angle
 *
 * @param angle in degree
 * @return sinus of 'angle'. sin(-90) = -32767, sin(90) = 32767
 */
int16_t EPIC_TrigoSin(int16_t angle)
{
    int16_t ret = 0;

    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;

    if (angle < 90)
    {
        ret = sin0_90_table[angle];
    }
    else if ((angle >= 90) && (angle < 180))
    {
        angle = 180 - angle;
        ret   = sin0_90_table[angle];
    }
    else if ((angle >= 180) && (angle < 270))
    {
        angle = angle - 180;
        ret   = -sin0_90_table[angle];
    }
    else
    {
        /*angle >=270*/
        angle = 360 - angle;
        ret   = -sin0_90_table[angle];
    }

    return ret;
}

/** Return with sinus of an angle in 0.1 degree
 *
 * @param angle in 0.1 degree
 * @return sinus of 'angle'. sin(-900) = -32767, sin(900) = 32767
 */
void EPIC_TrigoSinCosP1(int16_t angle, int16_t *sin_val, int16_t *cos_val)
{
    while (angle < 0) angle += 3600;
    while (angle >= 3600) angle -= 3600;

    int32_t angle_low = angle / 10;
    int32_t angle_hight = angle_low + 1;
    int32_t angle_rem = angle  - (angle_low * 10);

    if (sin_val)
    {
        int32_t s1 = EPIC_TrigoSin(angle_low);
        int32_t s2 = EPIC_TrigoSin(angle_hight);

        *sin_val = (s1 * (10 - angle_rem) + s2 * angle_rem) / 10;
    }
    if (cos_val)
    {
        int32_t c1 = EPIC_TrigoSin(angle_low + 90);
        int32_t c2 = EPIC_TrigoSin(angle_hight + 90);

        *cos_val = (c1 * (10 - angle_rem) + c2 * angle_rem) / 10;
    }
}


/**
 * @brief Calculate Intersectarea of given layers, Set res_area NULL if you just want to know if there have intersectarea.
 * @param layer0 -
 * @param layer1 -
 * @param res_area -
 * @return
 */
static bool EPIC_CalcIntersectArea(const EPIC_BlendingDataType *layer0, const EPIC_BlendingDataType *layer1, EPIC_AreaTypeDef *res_area)
{
    int16_t x0;
    int16_t y0;
    int16_t x1;
    int16_t y1;

    if ((0 == layer0->width)  || (0 == layer0->height) || (0 == layer1->width) || (0 == layer1->height)) return false;

    /* calculate the intersect region */
    x0 = HAL_MAX(layer0->x_offset, layer1->x_offset);
    y0 = HAL_MAX(layer0->y_offset, layer1->y_offset);
    x1 = HAL_MIN(layer0->x_offset + layer0->width - 1, layer1->x_offset + layer1->width - 1);
    y1 = HAL_MIN(layer0->y_offset + layer0->height - 1, layer1->y_offset + layer1->height - 1);

    if (res_area != NULL)
    {
        res_area->x0 = x0;
        res_area->y0 = y0;
        res_area->x1 = x1;
        res_area->y1 = y1;
    }

    if ((x0 <= x1) && (y0 <= y1))
    {
        return true;
    }
    else
    {
        return false;
    }
}


#ifdef SF32LB55X
/**
 * @brief Join areas of given layers
 * @param layer0 -
 * @param layer1 -
 * @param res_area -
 * @return
 */
static void EPIC_JoinArea(const EPIC_BlendingDataType *layer0, const EPIC_BlendingDataType *layer1, EPIC_AreaTypeDef *res_area)
{
    HAL_ASSERT(res_area != NULL);

    res_area->x0 = HAL_MIN(layer0->x_offset, layer1->x_offset);
    res_area->y0 = HAL_MIN(layer0->y_offset, layer1->y_offset);
    res_area->x1 = HAL_MAX(layer0->x_offset + layer0->width - 1, layer1->x_offset + layer1->width - 1);
    res_area->y1 = HAL_MAX(layer0->y_offset + layer0->height - 1, layer1->y_offset + layer1->height - 1);
}
#endif /* EPIC_UNUSED_FUNCTION */

static void AreaFromLayer(EPIC_AreaTypeDef *area, const EPIC_BlendingDataType *p_layer)
{
    area->x0 = p_layer->x_offset;
    area->y0 = p_layer->y_offset;
    area->x1 = p_layer->x_offset + p_layer->width - 1;
    area->y1 = p_layer->y_offset + p_layer->height - 1;
}

void HAL_EPIC_AreaMove(EPIC_AreaTypeDef *area, int16_t x, int16_t y)
{
    area->x0 += x;
    area->x1 += x;

    area->y0 += y;
    area->y1 += y;
}
#define AreaMove HAL_EPIC_AreaMove
bool HAL_EPIC_AreaIntersect(EPIC_AreaTypeDef *res_p, const EPIC_AreaTypeDef *a1_p, const EPIC_AreaTypeDef *a2_p)
{
    /*Get the smaller area from 'a1_p' and 'a2_p'*/
    res_p->x0 = HAL_MAX(a1_p->x0, a2_p->x0);
    res_p->y0 = HAL_MAX(a1_p->y0, a2_p->y0);
    res_p->x1 = HAL_MIN(a1_p->x1, a2_p->x1);
    res_p->y1 = HAL_MIN(a1_p->y1, a2_p->y1);

    /*If x0 or y0 greater than x1 or y1 then the areas union is empty*/
    bool union_ok = true;
    if ((res_p->x0 > res_p->x1) || (res_p->y0 > res_p->y1))
    {
        union_ok = false;
    }

    return union_ok;
}
/**
 * Check if an area is fully on an other
 * @param ain_p pointer to an area which could be in 'aholder_p'
 * @param aholder_p pointer to an area which could involve 'ain_p'
 * @return true: `ain_p` is fully inside `aholder_p`
 */
bool HAL_EPIC_AreaIsIn(const EPIC_AreaTypeDef *ain_p, const EPIC_AreaTypeDef *aholder_p)
{
    bool is_in = false;

    if (ain_p->x0 >= aholder_p->x0 && ain_p->y0 >= aholder_p->y0 && ain_p->x1 <= aholder_p->x1 &&
            ain_p->y1 <= aholder_p->y1)
    {
        is_in = true;
    }

    return is_in;
}

void HAL_EPIC_AreaJoin(EPIC_AreaTypeDef *a_res_p, const EPIC_AreaTypeDef *a1_p, const EPIC_AreaTypeDef *a2_p)
{
    a_res_p->x0 = HAL_MIN(a1_p->x0, a2_p->x0);
    a_res_p->y0 = HAL_MIN(a1_p->y0, a2_p->y0);
    a_res_p->x1 = HAL_MAX(a1_p->x1, a2_p->x1);
    a_res_p->y1 = HAL_MAX(a1_p->y1, a2_p->y1);
}


static void EPIC_ENABLE(EPIC_HandleTypeDef *hepic)
{
    if (!IS_REAL_INSTANCE(hepic->Instance)) return;

    hepic->start_tick = HAL_DBG_DWT_GetCycles();
    HAL_RCC_EnableModule(RCC_MOD_EPIC);
#ifdef EPIC_PERF_CNT_VAL
    hepic->Instance->PERF_CNT = 0;
#endif /* EPIC_PERF_CNT_VAL */
}

static void EPIC_DISABLE(EPIC_HandleTypeDef *hepic)
{
    if (!IS_REAL_INSTANCE(hepic->Instance)) return;

#ifdef EPIC_PERF_CNT_VAL
    uint32_t hw_cnt = hepic->Instance->PERF_CNT;
#endif /* EPIC_PERF_CNT_VAL */

    HAL_RCC_DisableModule(RCC_MOD_EPIC);
    hepic->end_tick = HAL_DBG_DWT_GetCycles();

#ifdef EPIC_PERF_CNT_VAL
    hepic->PerfCnt += hw_cnt;
    hepic->HalCnt  += HAL_GetElapsedTick(hepic->start_tick, hepic->end_tick) - hw_cnt;
#endif /* EPIC_PERF_CNT_VAL */
}

static uint32_t EPIC_GetColorDepth(uint32_t color_mode)
{
    uint32_t color_depth;

    if (EPIC_COLOR_RGB565 == color_mode)
    {
        color_depth = 16;
    }
    else if (EPIC_COLOR_ARGB8565 == color_mode)
    {
        color_depth = 24;
    }
    else if (EPIC_COLOR_RGB888 == color_mode)
    {
        color_depth = 24;
    }
    else if ((EPIC_COLOR_A8 == color_mode) || (EPIC_COLOR_MONO == color_mode))
    {
        color_depth = 8;
    }
    else if (EPIC_COLOR_A4 == color_mode)
    {
        color_depth = 4;
    }
    else if (EPIC_COLOR_A2 == color_mode)
    {
        color_depth = 2;
    }
    else if (EPIC_COLOR_L8 == color_mode)
    {
        color_depth = 8;
    }
    else if (EPIC_COLOR_ARGB8888 == color_mode)
    {
        color_depth = 32;
    }
    else if (IS_EZIP_COLOR_MODE(color_mode))
    {
        color_depth = 32;
    }
    else if (IS_YUV_COLOR_MODE(color_mode))
    {
        color_depth = 24;
    }
    else
    {
        HAL_ASSERT(0);
    }

    return color_depth;
}

uint32_t HAL_EPIC_GetColorDepth(uint32_t color_mode)
{
    return EPIC_GetColorDepth(color_mode);
}


#ifdef EPIC_SUPPORT_DITHER
static uint32_t EPIC_GetColorBitMask(uint32_t color_mode, Alpha_BlendTypeDef blend_mode)
{
    /*
        bit31~24 Alpha
        bit23~16 R
        bit15~8  G
        bit7~0   B
    */
    uint32_t bit_mask;

    if (EPIC_COLOR_RGB565 == color_mode)
    {
        bit_mask = 0x00F8FCF8;
    }
    else if (EPIC_COLOR_ARGB8565 == color_mode)
    {
        bit_mask = 0xFFF8FCF8;
    }
    else if (EPIC_COLOR_RGB888 == color_mode)
    {
        bit_mask = 0x00FFFFFF;
    }
    else if (EPIC_COLOR_A8 == color_mode)
    {
        if (IS_MASK_MODE(blend_mode))
            bit_mask = 0xFF000000;
        else
            bit_mask = 0xFFFFFFFF;
    }
    else if (EPIC_COLOR_A4 == color_mode)
    {
        if (IS_MASK_MODE(blend_mode))
            bit_mask = 0xF0000000;
        else
            bit_mask = 0xF0FFFFFF;
    }
    else if (EPIC_COLOR_A2 == color_mode)
    {
        if (IS_MASK_MODE(blend_mode))
            bit_mask = 0xC0000000;
        else
            bit_mask = 0xC0FFFFFF;
    }
    else if (IS_YUV_COLOR_MODE(color_mode))
    {
        bit_mask = 0x00FFFFFF;
    }
    else if ((EPIC_COLOR_ARGB8888 == color_mode)
             || (EPIC_COLOR_L8 == color_mode)
             || (EPIC_COLOR_MONO == color_mode))
    {
        bit_mask = 0xFFFFFFFF;
    }
    else
    {
        HAL_ASSERT(0);
    }

    return bit_mask;
}
#endif /* EPIC_SUPPORT_DITHER */

static uint32_t EPIC_GetLayerColorFormat(uint32_t color_mode)
{
    uint32_t layer_color_format;

    if (EPIC_COLOR_RGB565 == color_mode)
    {
        layer_color_format = EPIC_L0_CFG_FMT_RGB565;
    }
    else if (EPIC_COLOR_ARGB8565 == color_mode)
    {
        layer_color_format = EPIC_L0_CFG_FMT_ARGB8565;
    }
    else if (EPIC_COLOR_RGB888 == color_mode)
    {
        layer_color_format = EPIC_L0_CFG_FMT_RGB888;
    }
#ifndef SF32LB55X
    else if ((EPIC_COLOR_A8 == color_mode) || (EPIC_COLOR_MONO == color_mode))
    {
        layer_color_format = EPIC_L0_CFG_FMT_A8;
    }
    else if (EPIC_COLOR_A4 == color_mode)
    {
        layer_color_format = EPIC_L0_CFG_FMT_A4;
    }
    else if (EPIC_COLOR_L8 == color_mode)
    {
        layer_color_format = EPIC_L0_CFG_FMT_L8;
    }
#endif /* SF32LB55X */
#if !(defined(SF32LB55X)||defined(SF32LB58X)||defined(SF32LB56X))
    else if (EPIC_COLOR_A2 == color_mode)
    {
        layer_color_format = EPIC_L0_CFG_FMT_A2;
    }
#endif
    else if (EPIC_COLOR_ARGB8888 == color_mode)
    {
        layer_color_format = EPIC_L0_CFG_FMT_ARGB8888;
    }
    else if (IS_EZIP_COLOR_MODE(color_mode))
    {
        layer_color_format = EPIC_L0_CFG_FMT_ARGB8888; //Always set layer format to ARGB8888 at EZIP color mode
    }
    else if (IS_YUV_COLOR_MODE(color_mode))
    {
        layer_color_format = EPIC_L0_CFG_FMT_RGB888; //Always set layer format to RGB888 at yuv color mode
    }
    else
    {
        HAL_ASSERT(0);
    }

    return layer_color_format;
}

#ifdef EPIC_SUPPORT_MASK
static uint32_t EPIC_GetMaskLayerColorFormat(uint32_t color_mode)
{
    uint32_t layer_color_format;

    if ((EPIC_COLOR_A8 == color_mode) || (EPIC_COLOR_MONO == color_mode))
    {
        layer_color_format = EPIC_MASK_CFG_FMT_A8;
    }
    else if (EPIC_COLOR_A4 == color_mode)
    {
        layer_color_format = EPIC_MASK_CFG_FMT_A4;
    }
    else
    {
        HAL_ASSERT(0);
    }

    return layer_color_format;
}
#endif /* EPIC_SUPPORT_MASK */


static void EPIC_MixColor(EPIC_ColorDef *a, EPIC_ColorDef *b, EPIC_ColorDef *mix_c)
{
    mix_c->ch.alpha   = (a->ch.alpha + b->ch.alpha) >> 1;
    mix_c->ch.color_r = (a->ch.color_r + b->ch.color_r) >> 1;
    mix_c->ch.color_g = (a->ch.color_g + b->ch.color_g) >> 1;
    mix_c->ch.color_b = (a->ch.color_b + b->ch.color_b) >> 1;
}

static void EPIC_MirrorAreaByPivot(EPIC_AreaTypeDef *area, int16_t x)
{
    int16_t xx0, xx1;

    xx1 = EPIC_MIRROR_V(area->x0, x);
    xx0 = EPIC_MIRROR_V(area->x1, x);

    area->x1 = xx1;
    area->x0 = xx0;
}

static inline void EPIC_GetLayersCoordMin(EPIC_BlendingDataType *l1, EPIC_BlendingDataType *l2, EPIC_BlendingDataType *l3, int16_t *p_offset_x, int16_t *p_offset_y)
{
    *p_offset_x = INT16_MAX;
    *p_offset_y = INT16_MAX;

    if (l1)
    {
        if ((l1->width > 0) && (l1->height > 0)) //Skip invalid layer
        {
            *p_offset_x = EPIC_MATH_MIN(*p_offset_x, l1->x_offset);
            *p_offset_y = EPIC_MATH_MIN(*p_offset_y, l1->y_offset);
        }
    }
    if (l2)
    {

        if ((l2->width > 0) && (l2->height > 0))
        {
            *p_offset_x = EPIC_MATH_MIN(*p_offset_x, l2->x_offset);
            *p_offset_y = EPIC_MATH_MIN(*p_offset_y, l2->y_offset);
        }
    }
    if (l3)
    {
        if ((l3->width > 0) && (l3->height > 0))
        {
            *p_offset_x = EPIC_MATH_MIN(*p_offset_x, l3->x_offset);
            *p_offset_y = EPIC_MATH_MIN(*p_offset_y, l3->y_offset);
        }
    }

}

static inline void EPIC_MoveLayers(EPIC_BlendingDataType *l1, EPIC_BlendingDataType *l2, EPIC_BlendingDataType *l3, int16_t offset_x, int16_t offset_y)
{
    if ((offset_x != 0) || (offset_y != 0))
    {
        if (l1)
        {
            l1->x_offset += offset_x;
            l1->y_offset += offset_y;
        }
        if (l2)
        {
            l2->x_offset += offset_x;
            l2->y_offset += offset_y;
        }
        if (l3)
        {
            l3->x_offset += offset_x;
            l3->y_offset += offset_y;
        }
    }
}


static void EPIC_MakeAllLayerCoordValid(EPIC_BlendingDataType *l1, EPIC_BlendingDataType *l2, EPIC_BlendingDataType *l3)
{
    int16_t offset_x, offset_y;
    EPIC_GetLayersCoordMin(l1, l2, l3, &offset_x, &offset_y);
    EPIC_MoveLayers(l1, l2, l3, -offset_x, -offset_y);
}

static void EPIC_MakeAllLayerCoordValidEx(EPIC_BlendingDataType *layer, uint8_t layer_num)
{
    int16_t offset_x = INT16_MAX, offset_y = INT16_MAX;
    uint32_t i;

    for (i = 0; i < layer_num; i++)
    {
        if ((layer[i].width > 0) && (layer[i].height > 0)) //Skip invalid layer
        {
            offset_x = EPIC_MATH_MIN(offset_x, layer[i].x_offset);
            offset_y = EPIC_MATH_MIN(offset_y, layer[i].y_offset);
        }
    }

    offset_x = -offset_x;
    offset_y = -offset_y;

    if ((offset_x != 0) || (offset_y != 0))
    {
        for (i = 0; i < layer_num; i++)
        {
            layer[i].x_offset += offset_x;
            layer[i].y_offset += offset_y;
        }
    }
}

static bool EPIC_ClipLayerSrcByOutput(
    EPIC_BlendingDataType *input_layer,
    EPIC_TransformCfgTypeDef *rot_cfg,
    const EPIC_BlendingDataType *output_layer,
    EPIC_TransformResultDef *trans_result)
{
    EPIC_AreaTypeDef input_layer_area, input_clip;
    EPIC_AreaTypeDef output_layer_area;


    EPIC_PRINTF("\r\nEPIC_ClipLayerSrcByOutput \r\n");
    EPIC_DEBUG_PRINT_LAYER_INFO(input_layer, "input");
    EPIC_DEBUG_PRINT_LAYER_INFO(output_layer, "output");


    uint32_t color_depth;
    uint32_t x_round_v, y_round_v; //Clip pixel align value
    int16_t ext_x = 0;
    int16_t ext_y = 0;
    uint8_t v_mirror_enabled = 0;

    if (IS_YUV_COLOR_MODE(input_layer->color_mode))
    {
        if (EPIC_COLOR_YUV420_PLANAR == input_layer->color_mode)
        {
            x_round_v = 2;
            y_round_v = 2;
        }
        else
        {
            x_round_v = 2;
            y_round_v = 1;
        }
    }
    else
    {
        color_depth = EPIC_GetColorDepth(input_layer->color_mode);
        x_round_v = (color_depth < 8) ? (8 / (color_depth & 0x7)) : 1; //Clip pixel align value
        y_round_v = 1;
    }

    if (input_layer->x_offset_frac != 0) ext_x = 1;
    if (input_layer->y_offset_frac != 0) ext_y = 1;


    AreaFromLayer(&output_layer_area, output_layer);
    AreaMove(&output_layer_area, -input_layer->x_offset, -input_layer->y_offset);
    /*output_layer_area is base on input_layer's TL now */

    if (rot_cfg)
    {
        AreaMove(&output_layer_area, -rot_cfg->pivot_x, -rot_cfg->pivot_y);
        /*output_layer_area is base on pivot now*/

        if (1 == rot_cfg->h_mirror)
        {
            int16_t x0 = output_layer_area.x0;
            int16_t x1 = output_layer_area.x1;

            output_layer_area.x0 = EPIC_MIRROR_V(x1, 0);
            output_layer_area.x1 = EPIC_MIRROR_V(x0, 0);
        }

        if (1 == rot_cfg->v_mirror)
        {
            int16_t y0 = output_layer_area.y0;
            int16_t y1 = output_layer_area.y1;

            output_layer_area.y0 = EPIC_MIRROR_V(y1, 0);
            output_layer_area.y1 = EPIC_MIRROR_V(y0, 0);
            v_mirror_enabled = 1;
        }


        EPIC_DEBUG_PRINT_AREA_INFO(&output_layer_area, "output_layer_area(base on pivot)");

        if ((EPIC_INPUT_SCALING_FACTOR_1 != rot_cfg->scale_x) || (EPIC_INPUT_SCALING_FACTOR_1 != rot_cfg->scale_y))
        {
            uint32_t epic_pitch_x;
            uint32_t epic_pitch_y;

#ifdef SF32LB55X
            /*
                On 55x scaling down value will re-calculate by layer width&height,
                clipping source layer maybe tearing the image.

                So clipping layer when scaling up or coodinates overflow.
            */
            EPIC_AreaTypeDef res_area;
            EPIC_JoinArea(input_layer, output_layer, &res_area);
            if (((EPIC_INPUT_SCALING_FACTOR_1 < rot_cfg->scale_x) || (EPIC_INPUT_SCALING_FACTOR_1 < rot_cfg->scale_y))
                    && (!EPCI_IS_TLBR_OVERFLOW(EPIC_L0, res_area.x0, res_area.y0, res_area.x1, res_area.y1)))
            {
                return false;
            }
#endif /* SF32LB55X */
            epic_pitch_x = EPIC_CONV_SCALE_FACTOR(rot_cfg->scale_x);
            epic_pitch_y = EPIC_CONV_SCALE_FACTOR(rot_cfg->scale_y);

            /*
                Add '1' pitch pixels around of source layer, to make sure scaling get enough source data.
             */
            ext_x += 1;
            ext_y += 1;

            output_layer_area.x0 = EPIC_RSCALE_INT16(output_layer_area.x0 - ext_x, epic_pitch_x, -(EPIC_SCALE_1 - 1))/*round up RSCALE value*/;
            output_layer_area.x1 = EPIC_RSCALE_INT16(output_layer_area.x1 + ext_x, epic_pitch_x, EPIC_SCALE_1 - 1);
            output_layer_area.y0 = EPIC_RSCALE_INT16(output_layer_area.y0 - ext_y, epic_pitch_y, -(EPIC_SCALE_1 - 1));
            output_layer_area.y1 = EPIC_RSCALE_INT16(output_layer_area.y1 + ext_y, epic_pitch_y, EPIC_SCALE_1 - 1);

            EPIC_DEBUG_PRINT_AREA_INFO(&output_layer_area, "output_layer_area before scale(base on pivot)");

            ext_x = 0;
            ext_y = 0;
        }

        if (0 != rot_cfg->angle)
        {
            EPIC_PointTypeDef pivot_o;

            //Calculate pivot base on output_layer(output_layer is base on pivot now)
            pivot_o.x = 0 - output_layer_area.x0;
            pivot_o.y = 0 - output_layer_area.y0;

            EPIC_GetRotatedArea(&output_layer_area,
                                output_layer_area.x1 - output_layer_area.x0 + 1,
                                output_layer_area.y1 - output_layer_area.y0 + 1,
                                3600 - rot_cfg->angle, &pivot_o);

            //'EPIC_GetRotatedArea' return area is base on output_layer's TL move to pivot
            AreaMove(&output_layer_area, -pivot_o.x, -pivot_o.y);
            EPIC_DEBUG_PRINT_AREA_INFO(&output_layer_area, "output_layer_area before rotate(base on pivot)");
        }


        AreaMove(&output_layer_area, rot_cfg->pivot_x, rot_cfg->pivot_y);
        /*output_layer_area is base on input_layer's TL now*/
    }

    if (ext_x != 0)
    {
        output_layer_area.x0 = output_layer_area.x0 - ext_x;
        output_layer_area.x1 = output_layer_area.x1 + ext_x;
    }
    if (ext_y != 0)
    {
        output_layer_area.y0 = output_layer_area.y0 - ext_y;
        output_layer_area.y1 = output_layer_area.y1 + ext_y;
    }

    if (x_round_v > 1)
    {
        output_layer_area.x0 = HAL_ALIGN_DOWN(output_layer_area.x0, x_round_v);
        //output_layer_area.x1 = HAL_ALIGN(output_layer_area.x1 + 1, x_round_v) - 1;
    }
    if (y_round_v > 1)
    {
        output_layer_area.y0 = HAL_ALIGN_DOWN(output_layer_area.y0, y_round_v);
        if (v_mirror_enabled) output_layer_area.y1 = HAL_ALIGN(output_layer_area.y1 + 1, y_round_v) - 1;
    }

    AreaFromLayer(&input_layer_area, input_layer);
    AreaMove(&input_layer_area, -input_layer->x_offset, -input_layer->y_offset);
    /*input_layer_area is base on input_layer's TL now*/

    EPIC_DEBUG_PRINT_AREA_INFO(&output_layer_area, "output_layer_area(base on input's TL)");
    EPIC_DEBUG_PRINT_AREA_INFO(&input_layer_area,  "input_layer_area(base on input's TL)");

    if (HAL_EPIC_AreaIntersect(&input_clip, &input_layer_area, &output_layer_area))
    {
        if (IS_EZIP_COLOR_MODE(input_layer->color_mode))
        {
            HAL_ASSERT(!v_mirror_enabled);//Not supported
        }
        else
        {
            int32_t tgt_x, tgt_y;

            tgt_x = input_clip.x0;
            //Move to the beginning of last line if v_mirror_enabled
            tgt_y = v_mirror_enabled ? input_clip.y1 : input_clip.y0;
            HAL_ASSERT(0 == (tgt_x & (x_round_v - 1)));
            HAL_ASSERT(0 == (tgt_y & (y_round_v - 1)));
            //To avoid misaligned total_width of A2,A4 format
            uint16_t algined_total_w = HAL_ALIGN(input_layer->total_width, x_round_v);
            if (IS_YUV_COLOR_MODE(input_layer->color_mode))
            {
#ifdef EPIC_SUPPORT_YUV
                if (EPIC_COLOR_YUV420_PLANAR == input_layer->color_mode)
                {
                    input_layer->yuv.y_buf += (tgt_y * algined_total_w + tgt_x) * 1;
                    input_layer->yuv.u_buf += ((tgt_y >> 1) * (algined_total_w >> 1) + (tgt_x >> 1)) * 1;
                    input_layer->yuv.v_buf += ((tgt_y >> 1) * (algined_total_w >> 1) + (tgt_x >> 1)) * 1;
                }
                else
                {
                    input_layer->yuv.y_buf += (tgt_y * algined_total_w + tgt_x) * 2;
                }
#endif /* EPIC_SUPPORT_YUV */
            }
            else
            {
                input_layer->data += (HAL_ALIGN((tgt_y * algined_total_w + tgt_x) * color_depth, 8) >> 3);
            }

        }

        //Clip source image and update pivot
        input_layer->x_offset += input_clip.x0;
        input_layer->y_offset += input_clip.y0;
        input_layer->width    = input_clip.x1 - input_clip.x0 + 1;
        input_layer->height   = input_clip.y1 - input_clip.y0 + 1;

        trans_result->clip_area = input_clip;

        if (rot_cfg)
        {
            rot_cfg->pivot_x -= input_clip.x0;
            rot_cfg->pivot_y -= input_clip.y0;
        }

        EPIC_DEBUG_PRINT_AREA_INFO(&input_clip, "Final clip area(base on input's TL)");
    }
    else
    {
        input_layer->width  = 0;
        input_layer->height = 0;
    }

    EPIC_DEBUG_PRINT_LAYER_INFO(input_layer, "input(clipped)");
    return true;
}


/**
 * @brief Get area of VL which will be mirror and visible on canvas, and it is coordinate is base on video layer's TL
 * @param vl -
 * @param canvas -
 * @param pivot -  same origin with VL and CANVAS
 * @param vl_new_area -
 * @return
 */
static void EPIC_GetMirrorArea(EPIC_BlendingDataType *vl, const EPIC_BlendingDataType *canvas, const EPIC_PointTypeDef *pivot, EPIC_AreaTypeDef *vl_new_area)
{
    bool has_intersect;
    EPIC_AreaTypeDef  intersrc_area;

    EPIC_DEBUG_PRINT_LAYER_INFO(vl, "Fg layer");
    EPIC_DEBUG_PRINT_LAYER_INFO(canvas, "canvas layer");

    //Mirror video layer
    vl->x_offset = EPIC_MIRROR_V(vl->x_offset + vl->width - 1, pivot->x);

    EPIC_DEBUG_PRINT_LAYER_INFO(vl, "Mirrored Fg");


    has_intersect = EPIC_CalcIntersectArea(vl, canvas, &intersrc_area);

    EPIC_DEBUG_PRINT_AREA_INFO(&intersrc_area, "mirrored inter_area");

    //Revert video layer
    vl->x_offset = EPIC_MIRROR_V(vl->x_offset + vl->width - 1, pivot->x);
    EPIC_DEBUG_PRINT_LAYER_INFO(vl, "Revert Fg");

    if (has_intersect)
    {
        //Get area before mirrored
        EPIC_MirrorAreaByPivot(&intersrc_area, pivot->x);

        EPIC_DEBUG_PRINT_AREA_INFO(&intersrc_area, "before inter_area");

        HAL_ASSERT((intersrc_area.x0 <= intersrc_area.x1) && (intersrc_area.y0 <= intersrc_area.y1));
        HAL_ASSERT((intersrc_area.x0 >= vl->x_offset) && (intersrc_area.x0 <= vl->x_offset + vl->width - 1));
        HAL_ASSERT((intersrc_area.x1 >= vl->x_offset) && (intersrc_area.x1 <= vl->x_offset + vl->width - 1));

        vl_new_area->x0 = intersrc_area.x0 - vl->x_offset;
        vl_new_area->x1 = intersrc_area.x1 - vl->x_offset;
        vl_new_area->y0 = intersrc_area.y0 - vl->y_offset;
        vl_new_area->y1 = intersrc_area.y1 - vl->y_offset;

        EPIC_DEBUG_PRINT_AREA_INFO(vl_new_area, "VL inner inter_area");

    }
    else
    {
        memset(vl_new_area, 0, sizeof(EPIC_AreaTypeDef));
    }
}

/** Calculate area after rotation, area coordinate is relative to original topleft pixel(and pivot too)
 *  i.e. if angle=0, output is x0=0, y0=0, x1=w-1, y1=h-1
 *
 * @param[out] output
 * @param[in] w width
 * @param[in] h height
 * @param[in] angle angle in 0.1 degree
 * @param[in] pivot pivot
 * @return sinus of 'angle'. sin(-90) = -32767, sin(90) = 32767
 */
void EPIC_GetRotatedArea(EPIC_AreaTypeDef *output, uint16_t w, uint16_t h, int16_t angle,
                         const EPIC_PointTypeDef *pivot)
{
    int32_t angle_low = angle / 10;
    int32_t angle_hight = angle_low + 1;
    int32_t angle_rem = angle  - (angle_low * 10);

    int32_t s1 = EPIC_TrigoSin(angle_low);
    int32_t s2 = EPIC_TrigoSin(angle_hight);

    int32_t c1 = EPIC_TrigoSin(angle_low + 90);
    int32_t c2 = EPIC_TrigoSin(angle_hight + 90);

    int32_t sinma = (s1 * (10 - angle_rem) + s2 * angle_rem) / 10;
    int32_t cosma = (c1 * (10 - angle_rem) + c2 * angle_rem) / 10;

    EPIC_PointTypeDef lt;
    EPIC_PointTypeDef rt;
    EPIC_PointTypeDef lb;
    EPIC_PointTypeDef rb;

    int16_t xt;
    int16_t yt;

    EPIC_AreaTypeDef a;
    a.x0 = (-pivot->x);
    a.y0 = (-pivot->y);
    a.x1 = (w - pivot->x);
    a.y1 = (h - pivot->y);

    xt = a.x0;
    yt = a.y0;
    lt.x = ((cosma * xt - sinma * yt) >> EPIC_TRIGO_SHIFT) + pivot->x;
    lt.y = ((sinma * xt + cosma * yt) >> EPIC_TRIGO_SHIFT) + pivot->y;

    xt = a.x1;
    yt = a.y0;
    rt.x = ((cosma * xt - sinma * yt) >> EPIC_TRIGO_SHIFT) + pivot->x;
    rt.y = ((sinma * xt + cosma * yt) >> EPIC_TRIGO_SHIFT) + pivot->y;

    xt = a.x0;
    yt = a.y1;
    lb.x = ((cosma * xt - sinma * yt) >> EPIC_TRIGO_SHIFT) + pivot->x;
    lb.y = ((sinma * xt + cosma * yt) >> EPIC_TRIGO_SHIFT) + pivot->y;

    xt = a.x1;
    yt = a.y1;
    rb.x = ((cosma * xt - sinma * yt) >> EPIC_TRIGO_SHIFT) + pivot->x;
    rb.y = ((sinma * xt + cosma * yt) >> EPIC_TRIGO_SHIFT) + pivot->y;

    output->x0 = EPIC_MATH_MIN4(lb.x, lt.x, rb.x, rt.x) - 1;
    output->x1 = EPIC_MATH_MAX4(lb.x, lt.x, rb.x, rt.x) + 1;
    output->y0 = EPIC_MATH_MIN4(lb.y, lt.y, rb.y, rt.y) - 1;
    output->y1 = EPIC_MATH_MAX4(lb.y, lt.y, rb.y, rt.y) + 1;
}

/**
 * @brief Calculate point's coordinate after scaled base on expect_pivot.
 *  All coordinates below have SAME origin!
 *
 * @param point   -  [in & out]  Point to be scale
 * @param scale_x -  scale x
 * @param scale_y -  scale y
 * @param expect_pivot   -  Expect scale pivot
 * @param hw_pivot     -  Hardware scale pivot
 */
void EPIC_ScalePointByPivot(EPIC_PointTypeDef *point,
                            /* Q24.8 */
                            uint32_t scale_x,
                            /* Q24.8 */
                            uint32_t scale_y,
                            EPIC_PointTypeDef *expect_pivot,
                            EPIC_PointTypeDef *hw_pivot
                           )
{
    int32_t scale_pivot_delta_x = expect_pivot->x - hw_pivot->x;
    int32_t scale_pivot_delta_y = expect_pivot->y - hw_pivot->y;

    point->x += scale_pivot_delta_x - ((scale_pivot_delta_x * EPIC_SCALE_1) / ((int32_t)scale_x));
    point->y += scale_pivot_delta_y - ((scale_pivot_delta_y * EPIC_SCALE_1) / ((int32_t)scale_y));
}

#if defined(SF32LB56X)||defined(SF32LB52X)
/*Layer index to channel id, using at EZIP&YUV on butfly-lite*/
static uint32_t LayerIdx2CH(EPIC_LAYER_IDX layer_idx)
{
    uint32_t ch = UINT32_MAX;

    switch (layer_idx)
    {
    case EPIC_LAYER_IDX_VL:
        ch = 0;
        break;
    case EPIC_LAYER_IDX_0:
        ch = 1;
        break;
    case EPIC_LAYER_IDX_1:
        ch = 2;
        break;
    case EPIC_LAYER_IDX_2:
        ch = 3;
        break;
    default :
        HAL_ASSERT(0);
        break;
    }

    return ch;
}

/**
 * @brief config yuv engine, use layer.total_width as YUV image width
 * @param hepic -
 * @param layer_idx - which layer to be apply
 * @param config - yuv format, buffer config
 * @return
 */
static HAL_StatusTypeDef EPIC_Apply_YUV_on_layer(EPIC_HandleTypeDef *hepic,
        EPIC_LAYER_IDX layer_idx,
        EPIC_BlendingDataType *config)
{
    EPIC_TypeDef *epic = hepic->Instance;
    uint32_t Y_width;

    epic->COENG_CFG &= ~EPIC_COENG_CFG_YUV_EN_Msk;

    Y_width = config->total_width;

    switch (config->color_mode)
    {
    case EPIC_COLOR_YUV422_PACKED_YUYV:
    case EPIC_COLOR_YUV422_PACKED_UYVY:
    {
        uint32_t YUV_width;
        uint32_t fmt = (EPIC_COLOR_YUV422_PACKED_YUYV == config->color_mode) ? 0 : 1;

        YUV_width = Y_width * 2;

        epic->YUV_ENG_CFG0 = (0   << EPIC_YUV_ENG_CFG0_WIDTH_U_Pos) |
                             (YUV_width << EPIC_YUV_ENG_CFG0_WIDTH_Y_Pos) |
                             (fmt   << EPIC_YUV_ENG_CFG0_FORMAT_Pos);
        epic->YUV_ENG_CFG1 = (0 << EPIC_YUV_ENG_CFG1_WIDTH_V_Pos);

        epic->Y_SRC = (uint32_t) HCPU_MPI_SBUS_ADDR(config->yuv.y_buf);
        epic->U_SRC = 0;
        epic->V_SRC = 0;
    }
    break;

    case EPIC_COLOR_YUV420_PLANAR:
    {
        uint32_t U_V_width;
        uint32_t fmt = 2;

        U_V_width = Y_width >> 1;

        epic->YUV_ENG_CFG0 = (U_V_width   << EPIC_YUV_ENG_CFG0_WIDTH_U_Pos) |
                             (Y_width << EPIC_YUV_ENG_CFG0_WIDTH_Y_Pos) |
                             (fmt   << EPIC_YUV_ENG_CFG0_FORMAT_Pos);
        epic->YUV_ENG_CFG1 = (U_V_width << EPIC_YUV_ENG_CFG1_WIDTH_V_Pos);

        epic->Y_SRC = (uint32_t) HCPU_MPI_SBUS_ADDR(config->yuv.y_buf);
        epic->U_SRC = (uint32_t) HCPU_MPI_SBUS_ADDR(config->yuv.u_buf);
        epic->V_SRC = (uint32_t) HCPU_MPI_SBUS_ADDR(config->yuv.v_buf);
    }
    break;

    default:
        HAL_ASSERT(0);
        return HAL_ERROR;
    }

    MODIFY_REG(epic->COENG_CFG, EPIC_COENG_CFG_YUV_CH_SEL_Msk,
               MAKE_REG_VAL(LayerIdx2CH(layer_idx), EPIC_COENG_CFG_YUV_CH_SEL_Msk, EPIC_COENG_CFG_YUV_CH_SEL_Pos));

    epic->COENG_CFG |= EPIC_COENG_CFG_YUV_EN;

    EPIC_PRINTF("Apply YUV on layer %d", layer_idx);
    return HAL_OK;
}
#endif /* SF32LB56X */


#ifndef SF32LB55X
/**
 * @brief  Allocate free lookup table
 *
 * @param  epic EPIC instance
 *
 * @retval Free lookup table id
 */
static uint32_t EPIC_Allocate_L8Table(EPIC_TypeDef *epic)
{
#if (1 == EPIC_LOOKUP_TABLES)
    uint32_t free_tables[1] = {1};
#else
    uint32_t free_tables[2] = {1, 1};
#endif

    EPIC_LayerxTypeDef *layer_x;
    EPIC_VideoLayerxTypeDef *Vlayer_x;

    layer_x = (EPIC_LayerxTypeDef *)&epic->L0_CFG;
    for (uint32_t i = 0; i < 2; i++, layer_x++)
    {
        if ((EPIC_L0_CFG_ACTIVE | EPIC_L0_CFG_FMT_L8) == (layer_x->CFG & (EPIC_L0_CFG_ACTIVE_Msk | EPIC_L0_CFG_FORMAT_Msk)))
        {
            free_tables[(layer_x->MISC_CFG & EPIC_L0_MISC_CFG_CLUT_SEL_Msk) >> EPIC_L0_MISC_CFG_CLUT_SEL_Pos] = 0;
        }
    }

    Vlayer_x = (EPIC_VideoLayerxTypeDef *)&epic->VL_CFG;
    for (uint32_t i = 0; i < 2; i++, Vlayer_x++)
    {
        if ((EPIC_VL_CFG_ACTIVE | EPIC_VL_CFG_FMT_L8) == (Vlayer_x->CFG & (EPIC_VL_CFG_ACTIVE_Msk | EPIC_VL_CFG_FORMAT_Msk)))
        {
            free_tables[(layer_x->MISC_CFG & EPIC_VL_MISC_CFG_CLUT_SEL_Msk) >> EPIC_VL_MISC_CFG_CLUT_SEL_Pos] = 0;
        }
    }

    for (uint32_t i = 0; i < (sizeof(free_tables) / sizeof(free_tables[0])); i++)
    {
        if (free_tables[i]) return i;
    }

    return UINT32_MAX;
}

static HAL_StatusTypeDef EPIC_Overwrite_L8Table(EPIC_HandleTypeDef *hepic, uint32_t tab_id, uint8_t *pLTab, uint16_t LTab_Cnt)
{
    HAL_ASSERT(EPIC_LOOKUP_TABLES > tab_id);
    HAL_ASSERT(hepic->LTab[tab_id]);

    if (NULL == pLTab)
    {
        HAL_ASSERT(0);
        RETURN_ERROR(hepic, HAL_ERROR);
    }
    if (LTab_Cnt > EPIC_MAX_LOOKUP_TABLE_CNT) LTab_Cnt = EPIC_MAX_LOOKUP_TABLE_CNT;
    if (hepic->RamInstance_used)
    {
        hepic->RamLTab[tab_id] = pLTab;
        hepic->RamLTabSize[tab_id] = LTab_Cnt;
    }
    else
    {
        memcpy((void *)hepic->LTab[tab_id], pLTab, ((uint32_t)LTab_Cnt) << 2);
    }
    return HAL_OK;
}

/*Layer index to Mask layer channel id, using Mask function after butflypro*/
static uint32_t LayerIdx2MaskCh(EPIC_LAYER_IDX layer_idx)
{
    HAL_ASSERT(layer_idx < EPIC_LAYER_IDX_MAX);
    return (uint32_t)(1 << layer_idx);
}
#endif /* SF32LB55X */

static bool EPIC_IsLayerActive(EPIC_TypeDef *epic, EPIC_LAYER_IDX layer_idx)
{
    EPIC_LayerxTypeDef *layer_x;
    HAL_ASSERT(layer_idx < EPIC_LAYER_IDX_MAX);

    if (layer_idx < EPIC_LAYER_IDX_VL)
    {
        layer_x = (EPIC_LayerxTypeDef *)&epic->L0_CFG;
        layer_x += layer_idx;

        return (EPIC_L0_CFG_ACTIVE == (layer_x->CFG & EPIC_L0_CFG_ACTIVE_Msk));
    }
    else
    {
        return (EPIC_VL_CFG_ACTIVE == (epic->VL_CFG & EPIC_VL_CFG_ACTIVE_Msk));
    }

}


/*
    Disable EZIP, YUV, Mask fucntion applied on this layer if it is present
*/
static HAL_StatusTypeDef EPIC_Disable_Func_on_layer(EPIC_TypeDef          *epic, EPIC_LAYER_IDX layer_idx)
{
#if defined(SF32LB56X)||defined(SF32LB52X)
    if (epic->COENG_CFG & EPIC_COENG_CFG_YUV_EN)
    {
        if (LayerIdx2CH(layer_idx) == GET_REG_VAL(epic->COENG_CFG, EPIC_COENG_CFG_YUV_CH_SEL_Msk, EPIC_COENG_CFG_YUV_CH_SEL_Pos))
        {
            epic->COENG_CFG &= ~EPIC_COENG_CFG_YUV_EN;
        }
    }

    if (epic->COENG_CFG & EPIC_COENG_CFG_EZIP_EN)
    {
        if (LayerIdx2CH(layer_idx) == GET_REG_VAL(epic->COENG_CFG, EPIC_COENG_CFG_EZIP_CH_SEL_Msk, EPIC_COENG_CFG_EZIP_CH_SEL_Pos))
        {
            epic->COENG_CFG &= ~EPIC_COENG_CFG_EZIP_EN;
        }
    }
#endif


#ifndef SF32LB55X
    if (epic->MASK_CFG & EPIC_MASK_CFG_ACTIVE)
    {
        if (LayerIdx2MaskCh(layer_idx) == GET_REG_VAL(epic->MASK_CFG, EPIC_MASK_CFG_L0_MASK_EN_Msk, EPIC_MASK_CFG_L0_MASK_EN_Pos))
        {
            epic->MASK_CFG &= ~EPIC_MASK_CFG_ACTIVE;
        }
    }
#endif

    return HAL_OK;
}

/**
 * @brief  Enable and configure normal layer
 *
 * @param  epic EPIC instance
 * @param  layer_idx layer index
 * @param  config config
 * @param  using_alpha whether layer alpha is used
 * @param  alpha alpha value when using_alpha is true
 *
 * @retval HAL status
 */
static HAL_StatusTypeDef EPIC_ConfigLayer(EPIC_HandleTypeDef *hepic, EPIC_LAYER_IDX layer_idx,
        EPIC_BlendingDataType *config,
        bool using_alpha, uint8_t alpha)
{
    EPIC_LayerxTypeDef *layer_x;
    uint32_t color_depth, line_bytes, output_line_bytes;
    uint32_t layer_color_format;
    EPIC_TypeDef *epic = hepic->Instance;

    if ((config->x_offset < 0)
            || (config->y_offset < 0))
    {
        RETURN_ERROR(hepic, HAL_ERROR);
    }
#if defined(SF32LB52X)
    HAL_ASSERT(layer_idx < EPIC_LAYER_IDX_1);
#elif !defined(SF32LB55X)
    HAL_ASSERT(layer_idx < EPIC_LAYER_IDX_2);
#else
    HAL_ASSERT(layer_idx < EPIC_LAYER_IDX_VL);
#endif

    //Check overflow
    if (EPCI_IS_TLBR_OVERFLOW(EPIC_L0, config->x_offset,
                              config->y_offset,
                              config->x_offset + config->width - 1,
                              config->y_offset + config->height - 1))
    {
        //Overflow
        EPIC_DEBUG_PRINT_LAYER_INFO(config, "Overflow");
        HAL_ASSERT(0);
        return HAL_ERROR;
    }

    layer_x = (EPIC_LayerxTypeDef *)&epic->L0_CFG;
    layer_x += layer_idx;
    layer_x->TL_POS = MAKE_REG_VAL(config->x_offset, EPIC_L0_TL_POS_X0_Msk, EPIC_L0_TL_POS_X0_Pos)
                      | MAKE_REG_VAL(config->y_offset, EPIC_L0_TL_POS_Y0_Msk, EPIC_L0_TL_POS_Y0_Pos);
    layer_x->BR_POS = MAKE_REG_VAL(config->x_offset + config->width - 1, EPIC_L0_BR_POS_X1_Msk, EPIC_L0_BR_POS_X1_Pos)
                      | MAKE_REG_VAL(config->y_offset + config->height - 1, EPIC_L0_BR_POS_Y1_Msk, EPIC_L0_BR_POS_Y1_Pos);

#ifndef SF32LB55X
    layer_x->SRC = HCPU_MPI_SBUS_ADDR(config->data);
#else
    layer_x->SRC = (uint32_t)config->data;
#endif

    color_depth = EPIC_GetColorDepth(config->color_mode);
    layer_color_format = EPIC_GetLayerColorFormat(config->color_mode);
    output_line_bytes = HAL_ALIGN(color_depth * config->width, 8) >> 3;

    if (output_line_bytes > EPIC_PREFETCH_THRESHOLD)
        layer_x->CFG = EPIC_L0_CFG_PREFETCH_EN;
    else
        layer_x->CFG = 0;

    if (using_alpha || (EPIC_COLOR_MONO == config->color_mode))
    {
        if ((EPIC_L0_CFG_FMT_RGB565 != layer_color_format) && (EPIC_L0_CFG_FMT_RGB888 != layer_color_format)
                && (EPIC_COLOR_MONO != config->color_mode))
        {
            layer_x->CFG |= MAKE_REG_VAL(1, EPIC_L0_CFG_ALPHA_BLEND_Msk, EPIC_L0_CFG_ALPHA_BLEND_Pos);
            layer_x->CFG |= MAKE_REG_VAL(alpha, EPIC_L0_CFG_ALPHA_Msk, EPIC_L0_CFG_ALPHA_Pos);
        }
        else
        {
            layer_x->CFG |= EPIC_L0_CFG_ALPHA_SEL;
            layer_x->CFG |= MAKE_REG_VAL(alpha, EPIC_L0_CFG_ALPHA_Msk, EPIC_L0_CFG_ALPHA_Pos);
        }
    }
    else
    {
        layer_x->CFG |= MAKE_REG_VAL(255, EPIC_L0_CFG_ALPHA_Msk, EPIC_L0_CFG_ALPHA_Pos);
    }
#ifdef HAL_EZIP_MODULE_ENABLED
    if (IS_EZIP_COLOR_MODE(config->color_mode))
    {
#if defined(SF32LB56X)||defined(SF32LB52X)
        MODIFY_REG(epic->COENG_CFG, EPIC_COENG_CFG_EZIP_CH_SEL_Msk,
                   MAKE_REG_VAL(LayerIdx2CH(layer_idx), EPIC_COENG_CFG_EZIP_CH_SEL_Msk, EPIC_COENG_CFG_EZIP_CH_SEL_Pos));
        epic->COENG_CFG |= EPIC_COENG_CFG_EZIP_EN;
#else
        layer_x->CFG |= EPIC_L0_CFG_EZIP_EN;
#endif
#ifdef SF32LB58X
        MODIFY_REG(layer_x->MISC_CFG, EPIC_L0_MISC_CFG_EZIP_SEL_Msk,
                   MAKE_REG_VAL(0, EPIC_L0_MISC_CFG_EZIP_SEL_Msk, EPIC_L0_MISC_CFG_EZIP_SEL_Pos));
#endif /* SF32LB58X */
    }
#endif /* HAL_EZIP_MODULE_ENABLED */

#if defined(SF32LB56X)||defined(SF32LB52X)
    if (IS_YUV_COLOR_MODE(config->color_mode))
    {
        HAL_StatusTypeDef err;

        err = EPIC_Apply_YUV_on_layer(hepic, layer_idx, config);
        if (err != HAL_OK)
        {
            return err;
        }
    }
#endif /* SF32LB56X */

    if (IS_Ax_COLOR_MODE(config->color_mode))
    {
        HAL_ASSERT(config->color_en);
#ifndef SF32LB55X
        layer_x->FILL = MAKE_REG_VAL(0x0, EPIC_L0_FILL_BG_MODE_Msk, EPIC_L0_FILL_BG_MODE_Pos)
                        | MAKE_REG_VAL(config->color_r, EPIC_L0_FILL_BG_R_Msk, EPIC_L0_FILL_BG_R_Pos)
                        | MAKE_REG_VAL(config->color_g, EPIC_L0_FILL_BG_G_Msk, EPIC_L0_FILL_BG_G_Pos)
                        | MAKE_REG_VAL(config->color_b, EPIC_L0_FILL_BG_B_Msk, EPIC_L0_FILL_BG_B_Pos);
#endif /* SF32LB55X */
    }
    else if (config->color_en)
    {
        layer_x->CFG |= EPIC_L0_CFG_FILTER_EN;
        layer_x->FILTER = MAKE_REG_VAL(0x0, EPIC_L0_FILTER_FILTER_MASK_Msk, EPIC_L0_FILTER_FILTER_MASK_Pos)
                          | MAKE_REG_VAL(config->color_r, EPIC_L0_FILTER_FILTER_R_Msk, EPIC_L0_FILTER_FILTER_R_Pos)
                          | MAKE_REG_VAL(config->color_g, EPIC_L0_FILTER_FILTER_G_Msk, EPIC_L0_FILTER_FILTER_G_Pos)
                          | MAKE_REG_VAL(config->color_b, EPIC_L0_FILTER_FILTER_B_Msk, EPIC_L0_FILTER_FILTER_B_Pos);
    }

#ifndef SF32LB55X
    if (EPIC_COLOR_L8 == config->color_mode)
    {
        uint32_t tab_id = EPIC_Allocate_L8Table(hepic->Instance);

        HAL_ASSERT(tab_id <= (EPIC_L0_MISC_CFG_CLUT_SEL_Msk >> EPIC_L0_MISC_CFG_CLUT_SEL_Pos));
        EPIC_Overwrite_L8Table(hepic, tab_id, config->lookup_table, config->lookup_table_size);
        layer_x->MISC_CFG &= ~EPIC_L0_MISC_CFG_CLUT_SEL_Msk;
        layer_x->MISC_CFG |= MAKE_REG_VAL(tab_id, EPIC_L0_MISC_CFG_CLUT_SEL_Msk, EPIC_L0_MISC_CFG_CLUT_SEL_Pos);
    }
#endif /* SF32LB55X */

    layer_x->CFG |= layer_color_format;
    if (EPIC_COLOR_MONO != config->color_mode)
    {
        line_bytes = HAL_ALIGN(color_depth * config->total_width, 8) >> 3;
        layer_x->CFG |= MAKE_REG_VAL(line_bytes, EPIC_L0_CFG_WIDTH_Msk, EPIC_L0_CFG_WIDTH_Pos);
    }
    else
    {
        ;//Keep layer width as 0
    }

#if defined(SF32LB56X)||defined(SF32LB52X)
    if (IS_YUV_COLOR_MODE(config->color_mode))
    {
        layer_x->CFG |= EPIC_L0_CFG_ACTIVE;
    }
    else
#endif
    {
        if (layer_x->SRC != 0)
            layer_x->CFG |= EPIC_L0_CFG_ACTIVE;
        else
            layer_x->CFG &= ~EPIC_L0_CFG_ACTIVE;
    }

    return HAL_OK;
}

/**
 * @brief  Configure normal layer in continue mode
 *
 * @param  epic EPIC instance
 * @param  idx - 0 - bg,  1 - fg,  2- mask
 * @param  config config
 *
 * @retval HAL status
 */
static HAL_StatusTypeDef EPIC_ContConfigLayer(EPIC_HandleTypeDef *hepic, uint32_t idx,
        const EPIC_LayerConfigTypeDef *config)
{
    EPIC_LayerxTypeDef *layer_x;
    uint32_t reg_v, line_bytes;
    EPIC_TypeDef *epic = hepic->Instance;

    int16_t x_offset = config->x_offset;
    int16_t y_offset = config->y_offset;

    if ((x_offset < 0)
            || (y_offset < 0))
    {
        RETURN_ERROR(hepic, HAL_ERROR);
    }

    EPIC_LAYER_IDX layer_idx = hepic->api_cfg.cont_cfg.epic_layer[idx];

#if defined(SF32LB52X)
    HAL_ASSERT(layer_idx < EPIC_LAYER_IDX_1);
#elif !defined(SF32LB55X)
    HAL_ASSERT(layer_idx < EPIC_LAYER_IDX_2);
#else
    HAL_ASSERT(layer_idx < EPIC_LAYER_IDX_VL);
#endif
    //Check overflow
    if (EPCI_IS_TLBR_OVERFLOW(EPIC_L0, x_offset,
                              y_offset,
                              x_offset + config->width - 1,
                              y_offset + config->height - 1))
    {
        //Overflow
        EPIC_DEBUG_PRINT_LAYER_INFO(config, "Overflow");
        HAL_ASSERT(0);
        return HAL_ERROR;
    }

    layer_x = (EPIC_LayerxTypeDef *)&epic->L0_CFG;
    layer_x += layer_idx;
    layer_x->TL_POS = MAKE_REG_VAL(x_offset, EPIC_L0_TL_POS_X0_Msk, EPIC_L0_TL_POS_X0_Pos)
                      | MAKE_REG_VAL(y_offset, EPIC_L0_TL_POS_Y0_Msk, EPIC_L0_TL_POS_Y0_Pos);
    layer_x->BR_POS = MAKE_REG_VAL(x_offset + config->width - 1, EPIC_L0_BR_POS_X1_Msk, EPIC_L0_BR_POS_X1_Pos)
                      | MAKE_REG_VAL(y_offset + config->height - 1, EPIC_L0_BR_POS_Y1_Msk, EPIC_L0_BR_POS_Y1_Pos);

#ifndef SF32LB55X
    layer_x->SRC = HCPU_MPI_SBUS_ADDR(config->data);
#else
    layer_x->SRC = (uint32_t)config->data;
#endif

    if (IS_Ax_COLOR_MODE(config->color_mode))
    {
        HAL_ASSERT(config->color_en);
#ifndef SF32LB55X
        layer_x->FILL = MAKE_REG_VAL(0x0, EPIC_L0_FILL_BG_MODE_Msk, EPIC_L0_FILL_BG_MODE_Pos)
                        | MAKE_REG_VAL(config->color_r, EPIC_L0_FILL_BG_R_Msk, EPIC_L0_FILL_BG_R_Pos)
                        | MAKE_REG_VAL(config->color_g, EPIC_L0_FILL_BG_G_Msk, EPIC_L0_FILL_BG_G_Pos)
                        | MAKE_REG_VAL(config->color_b, EPIC_L0_FILL_BG_B_Msk, EPIC_L0_FILL_BG_B_Pos);
#endif /* SF32LB55X */
    }


    line_bytes = HAL_ALIGN(hepic->api_cfg.cont_cfg.color_depth[idx] * config->total_width, 8) >> 3;

    reg_v = MAKE_REG_VAL(line_bytes, EPIC_L0_CFG_WIDTH_Msk, EPIC_L0_CFG_WIDTH_Pos)
            | MAKE_REG_VAL(hepic->api_cfg.cont_cfg.color_format[idx], EPIC_L0_CFG_FORMAT_Msk, EPIC_L0_CFG_FORMAT_Pos)
            | EPIC_L0_CFG_ACTIVE;


    if (!IS_NO_ALPHA_COLOR_MODE(config->color_mode))
    {
        reg_v |= MAKE_REG_VAL(1, EPIC_L0_CFG_ALPHA_BLEND_Msk, EPIC_L0_CFG_ALPHA_BLEND_Pos);
        reg_v |= MAKE_REG_VAL(config->alpha, EPIC_L0_CFG_ALPHA_Msk, EPIC_L0_CFG_ALPHA_Pos);
    }
    else
    {
        reg_v |= EPIC_L0_CFG_ALPHA_SEL;
        reg_v |= MAKE_REG_VAL(config->alpha, EPIC_L0_CFG_ALPHA_Msk, EPIC_L0_CFG_ALPHA_Pos);
    }

    layer_x->CFG = reg_v;
    return HAL_OK;
}


/**
 * @brief  Disable normal layer
 *
 * @param  epic EPIC instance
 * @param  layer_idx layer index
 *
 * @retval HAL status
 */
static HAL_StatusTypeDef EPIC_DisableLayer(EPIC_TypeDef *epic, EPIC_LAYER_IDX layer_idx)
{
    EPIC_LayerxTypeDef *layer_x;

    HAL_ASSERT(layer_idx < EPIC_LAYER_IDX_VL);

    layer_x = (EPIC_LayerxTypeDef *)&epic->L0_CFG;
    layer_x += layer_idx;

    MODIFY_REG(layer_x->CFG, EPIC_L0_CFG_ACTIVE_Msk, 0);

#if defined(SF32LB56X)||defined(SF32LB52X)
    EPIC_Disable_Func_on_layer(epic, layer_idx);
#endif

    return HAL_OK;
}

static HAL_StatusTypeDef EPIC_DisableVideoLayer(EPIC_TypeDef *epic)
{
    /* clear VL status */
    epic->VL_ROT |= EPIC_VL_ROT_CALC_CLR;
    MODIFY_REG(epic->VL_CFG, EPIC_VL_CFG_ACTIVE_Msk, 0);
    //TODO: workaround as ezip_en is used even if active is false
#if defined(SF32LB56X)||defined(SF32LB52X)
    EPIC_Disable_Func_on_layer(epic, EPIC_LAYER_IDX_VL);
#else
    MODIFY_REG(epic->VL_CFG, EPIC_VL_CFG_EZIP_EN_Msk, 0);
#endif
#ifndef SF32LB55X
    MODIFY_REG(epic->L2_CFG, EPIC_L2_CFG_ACTIVE_Msk, 0);
    EPIC_Disable_Func_on_layer(epic, EPIC_LAYER_IDX_2);
#endif /* SF32LB55X */
    return HAL_OK;
}


/**
 * @brief  Configure output layer
 *
 * @param  epic EPIC instance
 * @param  fg foreground layer configuration
 * @param  bg background layer configuration
 * @param  dst output layer configuration
 *
 * @retval HAL status
 */
static HAL_StatusTypeDef EPIC_ConfigOutputLayer(EPIC_TypeDef *epic,
        EPIC_BlendingDataType *fg,
        EPIC_BlendingDataType *bg,
        EPIC_BlendingDataType *dst)
{
    uint8_t color_depth;
    int16_t x0;
    int16_t y0;
    int16_t x1;
    int16_t y1;
    uint32_t offset;
    uint32_t layer_color_format;

    if ((dst->x_offset < 0)
            || (dst->y_offset < 0))
    {
        return HAL_ERROR;
    }

    HAL_ASSERT(dst->total_width >= dst->width);

    if (EPIC_COLOR_RGB565 == dst->color_mode)
    {
        layer_color_format = EPIC_AHB_CTRL_O_FMT_RGB565;
        color_depth = 2;
    }
    else if (EPIC_COLOR_ARGB8565 == dst->color_mode)
    {
        layer_color_format = EPIC_AHB_CTRL_O_FMT_ARGB8565;
        color_depth = 3;
    }
    else if (EPIC_COLOR_RGB888 == dst->color_mode)
    {
        layer_color_format = EPIC_AHB_CTRL_O_FMT_RGB888;
        color_depth = 3;
    }
    else if (EPIC_COLOR_ARGB8888 == dst->color_mode)
    {
        layer_color_format = EPIC_AHB_CTRL_O_FMT_ARGB8888;
        color_depth = 4;
    }
    else
    {
        return HAL_ERROR;
    }

    if (bg->data != dst->data)
    {
        x0 = dst->x_offset;
        y0 = dst->y_offset;
        x1 = dst->x_offset + dst->width - 1;
        y1 = dst->y_offset + dst->height - 1;
        offset = 0;
    }
    else
    {
        EPIC_AreaTypeDef intersect_area;
        bool is_intersect;

        /* calculate the intersect region */
        is_intersect = EPIC_CalcIntersectArea(fg, dst, &intersect_area);

        if (is_intersect)
        {
            x0 = intersect_area.x0;
            y0 = intersect_area.y0;
            x1 = intersect_area.x1;
            y1 = intersect_area.y1;

            offset = ((y0 - dst->y_offset) * dst->total_width + (x0 - dst->x_offset)) * color_depth;
        }
        else
        {
            return HAL_EPIC_NOTHING_TO_DO;
        }
    }

    if ((x0 <= x1) && (y0 <= y1))
    {
        if ((!dst->color_en)   /* not filling using background color */
                && ((EPIC_COLOR_ARGB8565 == dst->color_mode)
                    || (EPIC_COLOR_ARGB8888 == dst->color_mode)))
        {
            /* don't blend with background color, output alpha directly */
            epic->CANVAS_BG |= EPIC_CANVAS_BG_BG_BLENDING_BYPASS;
        }
        else
        {
            if (dst->color_en)
            {
                epic->CANVAS_BG &= ~(EPIC_CANVAS_BG_RED_Msk | EPIC_CANVAS_BG_GREEN_Msk | EPIC_CANVAS_BG_BLUE_Msk);
                epic->CANVAS_BG |= MAKE_REG_VAL(dst->color_r, EPIC_CANVAS_BG_RED_Msk, EPIC_CANVAS_BG_RED_Pos)
                                   | MAKE_REG_VAL(dst->color_g, EPIC_CANVAS_BG_GREEN_Msk, EPIC_CANVAS_BG_GREEN_Pos)
                                   | MAKE_REG_VAL(dst->color_b, EPIC_CANVAS_BG_BLUE_Msk, EPIC_CANVAS_BG_BLUE_Pos);
            }

            epic->CANVAS_BG &= ~EPIC_CANVAS_BG_BG_BLENDING_BYPASS;
        }


        //Check overflow
        if (EPCI_IS_TLBR_OVERFLOW(EPIC_CANVAS, x0, y0, x1, y1))
        {
            //Overflow
            EPIC_DEBUG_PRINT_LAYER_INFO(dst, "Overflow");
            HAL_ASSERT(0);
            return HAL_ERROR;
        }

        epic->CANVAS_TL_POS = MAKE_REG_VAL(x0, EPIC_CANVAS_TL_POS_X0_Msk, EPIC_CANVAS_TL_POS_X0_Pos)
                              | MAKE_REG_VAL(y0, EPIC_CANVAS_TL_POS_Y0_Msk, EPIC_CANVAS_TL_POS_Y0_Pos);
        epic->CANVAS_BR_POS = MAKE_REG_VAL(x1, EPIC_CANVAS_BR_POS_X1_Msk, EPIC_CANVAS_BR_POS_X1_Pos)
                              | MAKE_REG_VAL(y1, EPIC_CANVAS_BR_POS_Y1_Msk, EPIC_CANVAS_BR_POS_Y1_Pos);

        epic->AHB_STRIDE = (dst->total_width - (x1 - x0 + 1)) * color_depth;
        MODIFY_REG(epic->AHB_CTRL,
                   EPIC_AHB_CTRL_DESTINATION_Msk | EPIC_AHB_CTRL_O_FORMAT_Msk,
                   EPIC_AHB_CTRL_DEST_RAM | layer_color_format);

        epic->AHB_MEM = (uint32_t)dst->data + offset;
        return HAL_OK;
    }
    else
    {
        return HAL_EPIC_NOTHING_TO_DO;
    }

}

static HAL_StatusTypeDef EPIC_ConfigOutputLayerEx(EPIC_HandleTypeDef *hepic,
        EPIC_BlendingDataType *input,
        uint8_t input_num,
        EPIC_BlendingDataType *output)
{
    uint8_t color_depth;
    int16_t x0;
    int16_t y0;
    int16_t x1;
    int16_t y1;
    uint32_t offset;
    uint32_t layer_color_format;

    if ((output->x_offset < 0)
            || (output->y_offset < 0))
    {
        RETURN_ERROR(hepic, HAL_ERROR);
    }
    HAL_ASSERT(output->total_width >= output->width);

    if (EPIC_COLOR_RGB565 == output->color_mode)
    {
        layer_color_format = EPIC_AHB_CTRL_O_FMT_RGB565;
        color_depth = 2;
    }
    else if (EPIC_COLOR_ARGB8565 == output->color_mode)
    {
        layer_color_format = EPIC_AHB_CTRL_O_FMT_ARGB8565;
        color_depth = 3;
    }
    else if (EPIC_COLOR_RGB888 == output->color_mode)
    {
        layer_color_format = EPIC_AHB_CTRL_O_FMT_RGB888;
        color_depth = 3;
    }
    else if (EPIC_COLOR_ARGB8888 == output->color_mode)
    {
        layer_color_format = EPIC_AHB_CTRL_O_FMT_ARGB8888;
        color_depth = 4;
    }
    else
    {
        RETURN_ERROR(hepic, HAL_ERROR);
    }

    if ((2 == input_num)
            && (input[0].data == output->data))
    {
        EPIC_AreaTypeDef intersect_area;
        bool is_intersect;

        /* calculate the intersect region */
        is_intersect = EPIC_CalcIntersectArea(&input[1], output, &intersect_area);

        if (is_intersect)
        {
            x0 = intersect_area.x0;
            y0 = intersect_area.y0;
            x1 = intersect_area.x1;
            y1 = intersect_area.y1;

            offset = ((y0 - output->y_offset) * output->total_width + (x0 - output->x_offset)) * color_depth;
        }
        else
        {
            return HAL_EPIC_NOTHING_TO_DO;
        }
    }
    else
    {
        x0 = output->x_offset;
        y0 = output->y_offset;
        x1 = output->x_offset + output->width - 1;
        y1 = output->y_offset + output->height - 1;
        offset = 0;
    }

    if ((x0 <= x1) && (y0 <= y1))
    {
        EPIC_TypeDef *epic = hepic->Instance;
        if ((!output->color_en)   /* not filling using background color */
                && ((EPIC_COLOR_ARGB8565 == output->color_mode)
                    || (EPIC_COLOR_ARGB8888 == output->color_mode)))
        {
            /* don't blend with background color, output alpha directly */
            epic->CANVAS_BG |= EPIC_CANVAS_BG_BG_BLENDING_BYPASS;
        }
        else
        {
            if (output->color_en)
            {
                epic->CANVAS_BG &= ~(EPIC_CANVAS_BG_RED_Msk | EPIC_CANVAS_BG_GREEN_Msk | EPIC_CANVAS_BG_BLUE_Msk);
                epic->CANVAS_BG |= MAKE_REG_VAL(output->color_r, EPIC_CANVAS_BG_RED_Msk, EPIC_CANVAS_BG_RED_Pos)
                                   | MAKE_REG_VAL(output->color_g, EPIC_CANVAS_BG_GREEN_Msk, EPIC_CANVAS_BG_GREEN_Pos)
                                   | MAKE_REG_VAL(output->color_b, EPIC_CANVAS_BG_BLUE_Msk, EPIC_CANVAS_BG_BLUE_Pos);
            }

            epic->CANVAS_BG &= ~EPIC_CANVAS_BG_BG_BLENDING_BYPASS;
        }

        //Check overflow
        if (EPCI_IS_TLBR_OVERFLOW(EPIC_CANVAS, x0, y0, x1, y1))
        {
            //Overflow
            EPIC_DEBUG_PRINT_LAYER_INFO(output, "Overflow");
            HAL_ASSERT(0);
            return HAL_ERROR;
        }

        epic->CANVAS_TL_POS = MAKE_REG_VAL(x0, EPIC_CANVAS_TL_POS_X0_Msk, EPIC_CANVAS_TL_POS_X0_Pos)
                              | MAKE_REG_VAL(y0, EPIC_CANVAS_TL_POS_Y0_Msk, EPIC_CANVAS_TL_POS_Y0_Pos);
        epic->CANVAS_BR_POS = MAKE_REG_VAL(x1, EPIC_CANVAS_BR_POS_X1_Msk, EPIC_CANVAS_BR_POS_X1_Pos)
                              | MAKE_REG_VAL(y1, EPIC_CANVAS_BR_POS_Y1_Msk, EPIC_CANVAS_BR_POS_Y1_Pos);

        epic->AHB_STRIDE = (output->total_width - (x1 - x0 + 1)) * color_depth;
        MODIFY_REG(epic->AHB_CTRL,
                   EPIC_AHB_CTRL_DESTINATION_Msk | EPIC_AHB_CTRL_O_FORMAT_Msk,
                   EPIC_AHB_CTRL_DEST_RAM | layer_color_format);

        epic->AHB_MEM = (uint32_t)output->data + offset;
        return HAL_OK;
    }
    else
    {
        return HAL_EPIC_NOTHING_TO_DO;
    }

}

static inline HAL_StatusTypeDef EPIC_ContConfigOutputLayer(EPIC_HandleTypeDef *hepic,
        EPIC_LayerConfigTypeDef *output)
{
    uint32_t color_bytes = hepic->api_cfg.cont_cfg.color_depth[CONT_BG_LAYER] >> 3;
    int16_t x0;
    int16_t y0;
    int16_t x1;
    int16_t y1;
    uint32_t offset;

    x0 = output->x_offset;
    y0 = output->y_offset;

    if ((x0 < 0) || (y0 < 0))
    {
        RETURN_ERROR(hepic, HAL_ERROR);
    }
    HAL_ASSERT(output->total_width >= output->width);

    x1 = x0 + output->width - 1;
    y1 = y0 + output->height - 1;
    offset = 0;

    if ((x0 <= x1) && (y0 <= y1))
    {
        EPIC_TypeDef *epic = hepic->Instance;

        //Check overflow
        if (EPCI_IS_TLBR_OVERFLOW(EPIC_CANVAS, x0, y0, x1, y1))
        {
            //Overflow
            EPIC_DEBUG_PRINT_LAYER_INFO(output, "Overflow");
            HAL_ASSERT(0);
            return HAL_ERROR;
        }

        epic->CANVAS_TL_POS = MAKE_REG_VAL(x0, EPIC_CANVAS_TL_POS_X0_Msk, EPIC_CANVAS_TL_POS_X0_Pos)
                              | MAKE_REG_VAL(y0, EPIC_CANVAS_TL_POS_Y0_Msk, EPIC_CANVAS_TL_POS_Y0_Pos);
        epic->CANVAS_BR_POS = MAKE_REG_VAL(x1, EPIC_CANVAS_BR_POS_X1_Msk, EPIC_CANVAS_BR_POS_X1_Pos)
                              | MAKE_REG_VAL(y1, EPIC_CANVAS_BR_POS_Y1_Msk, EPIC_CANVAS_BR_POS_Y1_Pos);

        epic->AHB_STRIDE = (output->total_width - (x1 - x0 + 1)) * color_bytes;

        epic->AHB_MEM = (uint32_t)output->data + offset;
        return HAL_OK;
    }
    else
    {
        return HAL_EPIC_NOTHING_TO_DO;
    }

}

#ifdef SF32LB55X

/**
 * @brief Optimize scale to sample more data and make even offset pixels after scale
 * @param p_pixels Pointer to pixels to scale
 * @param p_scale Pointer to scale, scale = factor*EPIC_INPUT_SCALE_NONE, EPIC_INPUT_SCALE_NONE means no scaling,
 */
static void rescale_and_align_2pixel(uint16_t *p_pixels, uint32_t *p_scale)
{
    uint32_t scale = *p_scale;
    uint32_t internal_scale;
    uint32_t new_pixels, old_pixels = *p_pixels;

    if (scale > EPIC_INPUT_SCALING_FACTOR_1)
    {
        new_pixels = (old_pixels * EPIC_INPUT_SCALING_FACTOR_1 / scale);
        if ((new_pixels & 1) != (old_pixels & 1))
        {
            new_pixels ++; //Offset is odd, make it even
        }
        if (new_pixels > 1)
        {
            scale = old_pixels * EPIC_INPUT_SCALING_FACTOR_1 / new_pixels; //Get more data of ending pixels
        }
    }

    internal_scale = EPIC_CONV_SCALE_FACTOR(scale);

    if ((internal_scale > 0) && (internal_scale <= EPIC_VL_SCALE_RATIO_PITCH_MAX))
    {
        *p_scale =  scale;
        *p_pixels = old_pixels * EPIC_SCALE_1 / internal_scale;
    }
    else
    {
        *p_pixels = old_pixels * EPIC_INPUT_SCALING_FACTOR_1 / *p_scale;
    }
}


#else

/**
 * @brief Get layer offset and scale_init value after fractional moving and scaling
 * @param pitch - Epic scale pitch in 16p16
 * @param layer_offset_out - layer offset value
 * @param new_pivot_out - Pivot base on scaled image's TL
 * @param old_pivot - Pivot base on src image's TL, in 16p16
 * @param scaleint_out - scale_init value
 * @param frac_move_in - Layer fractional coodinate move in 16p16 and must > 0
 */
void EPIC_scale_get_offset_scaleint(uint32_t pitch, int32_t old_pivot, int32_t frac_move_in,
                                    int16_t *layer_offset_out, uint32_t *scaleint_out,
                                    int32_t *new_pivot_out)
{
    int32_t new_pivot;
    int32_t new_pivot_int;
    int32_t new_pivot_frac;

    uint32_t scaleinit;
    int16_t  layer_offset;

    layer_offset = 0;
    new_pivot = EPIC_SCALE_INT32(old_pivot, pitch, 0);

    new_pivot_int  = new_pivot & 0xFFFF0000;
    new_pivot_frac = new_pivot - new_pivot_int;

    HAL_ASSERT((frac_move_in >= 0) && (frac_move_in < 0x10000));
    HAL_ASSERT((new_pivot_frac >= 0) && (new_pivot_frac < 0x10000));
    EPIC_PRINTF("new_pivot %x,  int=%x, frac=%x \r\n", new_pivot, new_pivot_int, new_pivot_frac);

    if (new_pivot_frac < frac_move_in)
    {
        scaleinit = EPIC_RSCALE_INT32((EPIC_SCALE_1 - frac_move_in + new_pivot_frac), pitch, 0);

        layer_offset  += 1; // 1 layer_offset == 1 pitch scaleinit
    }
    else
    {
        scaleinit = EPIC_RSCALE_INT32((new_pivot_frac - frac_move_in), pitch, 0);
    }

    layer_offset += (old_pivot - new_pivot_int) >> 16;



    *new_pivot_out  = new_pivot;
    *scaleint_out   = scaleinit;
    *layer_offset_out = layer_offset;

    EPIC_PRINTF("pitch=%x, old_pivot=%x,frac_move_in=%x, new_pivot=%x,scaleinit=%x,layer_offset=%x\r\n", \
                pitch, old_pivot, frac_move_in, new_pivot, scaleinit, layer_offset);




}
#endif /* !SF32LB55X */


/**
 * @brief  Calculate and set fg transform data to EPIC's tranform registers.
 *         And move fg&bg&dst to positive coordinates at end.

 * @param  epic EPIC instance
 * @param  rot_cfg rotation configuration
 * @param  rot_cfg transformation config
 * @param  fg configuration of foreground layer that's to be rotated
 * @param  dst output layer configuration
 * @param  trans_result transform calculation output result
 *
 * @retval HAL status
 */
static HAL_StatusTypeDef EPIC_TransformVideoLayer(EPIC_TypeDef *epic,
        EPIC_LAYER_IDX layer_idx,
        EPIC_TransformCfgTypeDef *rot_cfg,
        EPIC_BlendingDataType *fg,
        const EPIC_BlendingDataType *dst,
        EPIC_TransformResultDef *trans_result)
{
    /*
        new_offset_x&new_offset_y is video layer new  TL
    */
    int16_t new_offset_x;
    int16_t new_offset_y;
    /*
        (new_offset_x + new_width), (new_offset_y + new_height)  is Video layer BR

        new_width is fg->width-1
        new_height too.

    */
    uint16_t new_width;
    uint16_t new_height;

    uint32_t color_depth;
    int16_t angle_degree;


    uint32_t epic_scale_x;       //EPIC scale value
    uint32_t epic_scale_y;       //EPIC scale value
    EPIC_PointTypeDef pivot;                //Pivot base on VideoLayer's origin(or base on submodule's origin)

    EPIC_VideoLayerxTypeDef *Vlayer_x;
    EPIC_VideoLayerxTransTypeDef *Vlayer_x_trans;
    uint32_t vl_idx;
    Vlayer_x       = (EPIC_VideoLayerxTypeDef *)&epic->VL_CFG;
    Vlayer_x_trans = (EPIC_VideoLayerxTransTypeDef *)&epic->VL_ROT_M_CFG1;

#ifdef SF32LB58X //L2 support transfrom only on 58x
    HAL_ASSERT((EPIC_LAYER_IDX_VL == layer_idx) || (EPIC_LAYER_IDX_2 == layer_idx));
    vl_idx = (EPIC_LAYER_IDX_VL == layer_idx) ? 0 : 1;
#else
    HAL_ASSERT(EPIC_LAYER_IDX_VL == layer_idx);
    vl_idx = 0;
#endif /* SF32LB55X */

    Vlayer_x += vl_idx;
    Vlayer_x_trans += vl_idx;

    if ((0 == fg->width) || (0 == fg->height))
    {
        EPIC_DEBUG_PRINT_LAYER_INFO(fg, "Skip invalid video layer");
        return HAL_OK;
    }

    EPIC_PRINTF("\r\nEPIC_TransformVideoLayer angle=%d,scale_xy=%d,%d, pivot_xy=%d,%d\n",
                rot_cfg->angle, rot_cfg->scale_x, rot_cfg->scale_y,
                rot_cfg->pivot_x, rot_cfg->pivot_y);

    epic_scale_x = EPIC_CONV_SCALE_FACTOR(rot_cfg->scale_x);
    epic_scale_y = EPIC_CONV_SCALE_FACTOR(rot_cfg->scale_y);



    /* ensure angle reflect the actual quadrant, e.g. 90.1 and 91 are in same quadrant,
       quadrant of 0/90/180/270 degree is undefined */
    angle_degree = rot_cfg->angle / 10;
    if ((0 != rot_cfg->angle) && (0 == (angle_degree % 90)))
    {
        angle_degree += 1;
    }

    if (0 != rot_cfg->angle)
    {
        int16_t sinma;
        int16_t cosma;
        int16_t abs_sinma;
        int16_t abs_cosma;

        /* set rotation angle */
        EPIC_TrigoSinCosP1(rot_cfg->angle, &sinma, &cosma);

        abs_sinma = EPIC_MATH_ABS(sinma);
        abs_cosma = EPIC_MATH_ABS(cosma);

        trans_result->abs_sinma       = abs_sinma;
        trans_result->abs_cosma       = abs_cosma;
    }

    trans_result->angle_degree = angle_degree;
    trans_result->h_mirror = rot_cfg->h_mirror;
    trans_result->v_mirror = rot_cfg->v_mirror;


    color_depth = EPIC_GetColorDepth(fg->color_mode);

    /******************************************************************************/
    new_offset_x = fg->x_offset;
    new_offset_y = fg->y_offset;
    new_width    = fg->width - 1;
    new_height   = fg->height - 1;
    pivot.x      = rot_cfg->pivot_x;
    pivot.y      = rot_cfg->pivot_y;

    {

        /*
            Use video layer sub module to rotate and scale
            - sub module's TL is video layer's TL
            - it will clip by video layer's BR after transform(rotate/scale/mirror)

            http://gitlab.sifli.cc/sw/bt/issues/763#note_10243
        */
        uint16_t rotated_width;
        uint16_t rotated_height;
        EPIC_PointTypeDef origin;               //Video layer submodule orgin, it's TL equals video layer 's TL
        EPIC_AreaTypeDef rot_area;              //Image area after rotated, base on submodule's origin
        EPIC_PointTypeDef src_img;              //Src image 's coordinate before transformed, base on submodule's origin


        origin.x = 0;
        origin.y = 0;

        src_img.x = origin.x;
        src_img.y = origin.y;

        pivot.x = origin.x + src_img.x + rot_cfg->pivot_x;
        pivot.y = origin.y + src_img.y + rot_cfg->pivot_y;


        if (rot_cfg->angle != 0)
        {
            int32_t delta_x;
            int32_t delta_y;

            EPIC_GetRotatedArea(&rot_area, fg->width, fg->height, rot_cfg->angle, &pivot);


#ifdef SF32LB55X //pivot are not support negative
            /*Make sure rotated image is all visible(NOT be clipped by VideoLayer submodule)*/
            delta_x = EPIC_MATH_MIN(pivot.x, rot_area.x0);
            delta_y = EPIC_MATH_MIN(pivot.y, rot_area.y0);
#else
            delta_x = rot_area.x0;
            delta_y = rot_area.y0;
#endif /* SF32LB55X */

            if (delta_x < 0)
                delta_x = -delta_x;
            else
                delta_x = 0;

            if (delta_y < 0)
                delta_y = -delta_y;
            else
                delta_y = 0;

            pivot.x     += delta_x;
            pivot.y     += delta_y;
            src_img.x   += delta_x;
            src_img.y   += delta_y;
            AreaMove(&rot_area, delta_x, delta_y);


            //rotated_width  = rot_area.x1 - rot_area.x0 + 1;
            //rotated_height = rot_area.y1 - rot_area.y0 + 1;
            rotated_width  = EPIC_MATH_MAX(rot_area.x1, src_img.x + fg->width - 1)  - EPIC_MATH_MIN(rot_area.x0, 0);
            rotated_height = EPIC_MATH_MAX(rot_area.y1, src_img.y + fg->height - 1)  - EPIC_MATH_MIN(rot_area.y0, 0);


            EPIC_PRINTF("delta_x&y[%d,%d]\n", delta_x, delta_y);

            EPIC_PRINTF("rotate_pivot[%d,%d],  rot_area[(%d,%d),(%d,%d)]\n",
                        pivot.x, pivot.y,
                        rot_area.x0,
                        rot_area.y0,
                        rot_area.x1,
                        rot_area.y1
                       );

            EPIC_PRINTF("src_img[%d,%d],  rotate_pivot(%d,%d), w:%d, h:%d\n",
                        src_img.x,
                        src_img.y,
                        pivot.x,
                        pivot.y,
                        rotated_width,
                        rotated_height
                       );

            trans_result->rotated_width  = rotated_width;
            trans_result->rotated_height = rotated_height;
            trans_result->pivot.x = pivot.x;
            trans_result->pivot.y = pivot.y;
            trans_result->src_img.x = src_img.x;
            trans_result->src_img.y = src_img.y;

            /*
                Clip VideoLayer submodule image by moving videolayer's TL&BR
            */

            new_offset_x -= delta_x;
            new_offset_y -= delta_y;
            new_width    = rotated_width - 1;
            new_height   = rotated_height - 1;
        }
        else
        {
            /* disable manual first to make rot_max_col/line calculation correct if manual rotation is disabled */
            rotated_width  = fg->width - 1;
            rotated_height = fg->height - 1;;
        }

        //Scale
#ifndef SF32LB55X
        if ((epic_scale_x != EPIC_SCALE_1) || (epic_scale_y != EPIC_SCALE_1)
                || (fg->x_offset_frac != 0) || (fg->y_offset_frac != 0))
        {
            //Ajust Video layer's TLBR by scale
            uint32_t scale_init_x, scale_init_y;


            scale_init_x = 0;
            scale_init_y = 0;

            EPIC_PRINTF("scale before 16p16_x,y=[%x,%x]\n", EPIC_TO_INT32_COORD(new_offset_x, fg->x_offset_frac),
                        EPIC_TO_INT32_COORD(new_offset_y, fg->y_offset_frac));

            EPIC_PRINTF("epic_scale_x=0x%x,epic_scale_y=0x%x\n", epic_scale_x, epic_scale_y);
            EPIC_PRINTF("rotated_width=%d,rotated_width=%d\n", rotated_width, rotated_width);



            int16_t layer_offset;
            int32_t new_pivot_x, new_pivot_y;

            EPIC_scale_get_offset_scaleint(epic_scale_x,
                                           (((int32_t) pivot.x) << 16),
                                           fg->x_offset_frac,
                                           &layer_offset,
                                           &scale_init_x,
                                           &new_pivot_x);

            new_offset_x += layer_offset;


            EPIC_scale_get_offset_scaleint(epic_scale_y,
                                           (((int32_t) pivot.y) << 16),
                                           fg->y_offset_frac,
                                           &layer_offset,
                                           &scale_init_y,
                                           &new_pivot_y);

            new_offset_y += layer_offset;

            //Disable layer if scale_init or pitch out of src
            if ((((uint32_t)rotated_width << 16)  < scale_init_x) || (((uint32_t)rotated_width << 16)  < epic_scale_x))
            {
                fg->width = 0;
                goto __EXIT;
            }
            else
            {
                new_width =  EPIC_SCALE_INT32((((uint32_t)rotated_width << 16)  - scale_init_x), epic_scale_x, 0) >> EPIC_SCALE_FACTOR_FRAC_SIZE;
            }

            if ((((uint32_t)rotated_height << 16)  < scale_init_y) || (((uint32_t)rotated_height << 16)  < epic_scale_y))
            {
                fg->height = 0;
                goto __EXIT;
            }
            else
            {
                new_height = EPIC_SCALE_INT32((((uint32_t)rotated_height << 16)  - scale_init_y), epic_scale_y, 0) >> EPIC_SCALE_FACTOR_FRAC_SIZE;
            }

            pivot.x = new_pivot_x >> 16;
            pivot.y = new_pivot_y >> 16;

            EPIC_PRINTF("new w&h:%d,%d\n", new_width, new_height);




            EPIC_PRINTF("Bscale_init_x=%x,scale_init_y=%x\n", scale_init_x, scale_init_y);
            EPIC_PRINTF("scale after 16p16_x,y=[%x,%x]\n", EPIC_TO_INT32_COORD_INNER(new_offset_x, scale_init_x),
                        EPIC_TO_INT32_COORD_INNER(new_offset_y, scale_init_y));


            trans_result->scale_init_x = scale_init_x;
            trans_result->scale_init_y = scale_init_y;
        }
#else
        if ((epic_scale_x != EPIC_SCALE_1) || (epic_scale_y != EPIC_SCALE_1))
        {
            //Ajust Video layer's TLBR by scale
            int32_t new_pivot_x;
            int32_t new_pivot_y;

            uint32_t t_scale_x, t_scale_y;

            t_scale_x = rot_cfg->scale_x;
            t_scale_y = rot_cfg->scale_y;

            new_width = rotated_width;
            new_height = rotated_height;
            rescale_and_align_2pixel(&new_width, &t_scale_x);
            rescale_and_align_2pixel(&new_height, &t_scale_y);
            epic_scale_x = EPIC_CONV_SCALE_FACTOR(t_scale_x);
            epic_scale_y = EPIC_CONV_SCALE_FACTOR(t_scale_y);
            EPIC_PRINTF("\r\n after rescale_and_align_2pixel scale_xy=%x,%x, new_width,height=%d,%d\n",
                        epic_scale_x, epic_scale_y, new_width, new_height);

            new_pivot_x = (pivot.x * ((int32_t)EPIC_SCALE_1) + ((int32_t)epic_scale_x >> 1)) / ((int32_t)epic_scale_x);
            new_pivot_y = (pivot.y * ((int32_t)EPIC_SCALE_1) + ((int32_t)epic_scale_y >> 1)) / ((int32_t)epic_scale_y);

            new_offset_x += pivot.x - new_pivot_x;
            new_offset_y += pivot.y - new_pivot_y;

            pivot.x = new_pivot_x;
            pivot.y = new_pivot_y;
        }
#endif /* SF32LB55X */

    }


    //Hardware: mirror the intersection area of VL&CANVAS
    if (rot_cfg->h_mirror) //((rot_cfg->h_mirror) && (!rot_cfg->v_mirror))
    {
        //Mirror clipped video area
        int32_t layer_x0_16p16 = (((int32_t)new_offset_x) << 16) - ((int32_t) trans_result->scale_init_x);
        int32_t pivot_x_16p16 = ((int32_t) pivot.x) << 16;
        int32_t layer_x1_16p16 = layer_x0_16p16 + (((int32_t) new_width) << 16);

        layer_x0_16p16 = EPIC_MIRROR_V(layer_x1_16p16, layer_x0_16p16 + pivot_x_16p16);

        trans_result->scale_init_x = EPIC_SCALE_1 - (layer_x0_16p16 & 0xFFFF);
        new_offset_x = (int16_t)(layer_x0_16p16 >> 16);
        new_offset_x += (trans_result->scale_init_x != 0) ? 1 : 0;
    }

    if (rot_cfg->v_mirror) //((!rot_cfg->h_mirror) && (rot_cfg->v_mirror))
    {
        int16_t pivot_y_new, pivot_y_old; //Pivot base on {0,0}

        pivot_y_old  = fg->y_offset + pivot.y;
        pivot_y_new  = fg->y_offset + new_height - pivot.y;
        new_offset_y += pivot_y_old - pivot_y_new;
    }


    /******************************************************************************/
    fg->x_offset = new_offset_x;
    fg->y_offset = new_offset_y;
    fg->width = new_width + 1;
    fg->height = new_height + 1;


    if ((fg->width > (EPIC_LAYER_MAX_COORDINATE + 1))
            || (fg->height > (EPIC_LAYER_MAX_COORDINATE + 1)))
    {
        EPIC_DEBUG_PRINT_LAYER_INFO(fg, "Fg layer size Overflow");

        //Limit fg width&height in '0 ~ EPIC_LAYER_MAX_COORDINATE+1'
        fg->width  = EPIC_MATH_MIN(fg->width,  EPIC_LAYER_MAX_COORDINATE + 1);
        fg->height = EPIC_MATH_MIN(fg->height, EPIC_LAYER_MAX_COORDINATE + 1);
    }


    trans_result->epic_scale_x = epic_scale_x;
    trans_result->epic_scale_y = epic_scale_y;
    trans_result->h_mirror       = rot_cfg->h_mirror;
    trans_result->v_mirror       = rot_cfg->v_mirror;

__EXIT:

    EPIC_DEBUG_PRINT_LAYER_INFO(fg, "Fg layer(Transform end)");

    /******************************************************************************/

    return HAL_OK;
}


/**
 * @brief  Enable and configure video layer
 *
 * @param  epic EPIC instance
 * @param  layer_idx video layer index
 * @param  trans_result transformation config
 * @param  config layer configuration
 * @param  alpha alpha value
 * @param  layer_depth - 0:between BG&L0, 1:between L0&L1, 2:between L1&L2, 3:Over L2
 *
 * @retval HAL status
 */
static HAL_StatusTypeDef EPIC_ConfigVideoLayer(EPIC_HandleTypeDef *epic_handle,
        EPIC_LAYER_IDX layer_idx,
        const EPIC_TransformResultDef *trans_result,
        EPIC_BlendingDataType *config,
        uint8_t alpha, uint8_t layer_depth)
{

    EPIC_VideoLayerxTypeDef *Vlayer_x;
    EPIC_VideoLayerxTransTypeDef *Vlayer_x_trans;
    EPIC_TypeDef *epic = epic_handle->Instance;
    uint32_t vl_idx;


    if (EPIC_LAYER_IDX_VL == layer_idx)
    {
        HAL_ASSERT(layer_depth <= (EPIC_VL_CFG_BLEND_DEPTH_Msk >> EPIC_VL_CFG_BLEND_DEPTH_Pos));
    }

    //Check overflow
    if (EPCI_IS_TLBR_OVERFLOW(EPIC_VL, config->x_offset, config->y_offset, config->x_offset + config->width - 1, config->y_offset + config->height - 1))
    {
        //Overflow
        EPIC_DEBUG_PRINT_LAYER_INFO(config, "Overflow");
        HAL_ASSERT(0);
        return HAL_ERROR;
    }


    EPIC_DEBUG_PRINT_LAYER_INFO(config, "Video layer");
    EPIC_PRINTF("Video Layer depth %d \n", layer_depth);


    Vlayer_x       = (EPIC_VideoLayerxTypeDef *)&epic->VL_CFG;
    Vlayer_x_trans = (EPIC_VideoLayerxTransTypeDef *)&epic->VL_ROT_M_CFG1;


#ifndef SF32LB55X
    HAL_ASSERT((EPIC_LAYER_IDX_VL == layer_idx) || (EPIC_LAYER_IDX_2 == layer_idx));
    vl_idx = (EPIC_LAYER_IDX_VL == layer_idx) ? 0 : 1;
#else
    HAL_ASSERT(EPIC_LAYER_IDX_VL == layer_idx);
    vl_idx = 0;
#endif /* SF32LB55X */

    Vlayer_x += vl_idx;
    Vlayer_x_trans += vl_idx;



    HAL_ASSERT((trans_result->epic_scale_x <= EPIC_VL_SCALE_RATIO_XPITCH_MAX) && (trans_result->epic_scale_y <= EPIC_VL_SCALE_RATIO_YPITCH_MAX));

#ifndef SF32LB55X
    Vlayer_x->SRC = HCPU_MPI_SBUS_ADDR(config->data);
#else
    Vlayer_x->SRC = (uint32_t)config->data;
#endif


    uint32_t layer_color_format = EPIC_GetLayerColorFormat(config->color_mode);
    uint32_t color_depth        = EPIC_GetColorDepth(config->color_mode);
    uint32_t line_bytes = HAL_ALIGN(color_depth * config->total_width, 8) >> 3;

    {

        Vlayer_x->CFG = 0;

#ifdef HAL_EZIP_MODULE_ENABLED
        if (IS_EZIP_COLOR_MODE(config->color_mode))
        {
#if defined(SF32LB56X)||defined(SF32LB52X)
            MODIFY_REG(epic->COENG_CFG, EPIC_COENG_CFG_EZIP_CH_SEL_Msk,
                       MAKE_REG_VAL(LayerIdx2CH(layer_idx), EPIC_COENG_CFG_EZIP_CH_SEL_Msk, EPIC_COENG_CFG_EZIP_CH_SEL_Pos));
            epic->COENG_CFG |= EPIC_COENG_CFG_EZIP_EN;
#else
            Vlayer_x->CFG |= EPIC_VL_CFG_EZIP_EN;
#endif

#ifdef SF32LB58X
            MODIFY_REG(Vlayer_x->MISC_CFG, EPIC_VL_MISC_CFG_EZIP_SEL_Msk,
                       MAKE_REG_VAL(0, EPIC_VL_MISC_CFG_EZIP_SEL_Msk, EPIC_VL_MISC_CFG_EZIP_SEL_Pos));
#endif /* SF32LB55X */
        }
#endif /* HAL_EZIP_MODULE_ENABLED */

#if defined(SF32LB56X)||defined(SF32LB52X)
        if (IS_YUV_COLOR_MODE(config->color_mode))
        {
            HAL_StatusTypeDef err;

            err = EPIC_Apply_YUV_on_layer(epic_handle, layer_idx, config);
            if (err != HAL_OK)
            {
                return err;
            }
        }
#endif /* SF32LB56X */

        if ((EPIC_LAYER_OPAQUE != alpha) || (EPIC_COLOR_MONO == config->color_mode))
        {
            if (!IS_NO_ALPHA_COLOR_MODE(config->color_mode))
            {
                Vlayer_x->CFG |= MAKE_REG_VAL(1, EPIC_VL_CFG_ALPHA_BLEND_Msk, EPIC_VL_CFG_ALPHA_BLEND_Pos)
                                 | MAKE_REG_VAL(alpha, EPIC_VL_CFG_ALPHA_Msk, EPIC_VL_CFG_ALPHA_Pos);
            }
            else
            {
                Vlayer_x->CFG |= MAKE_REG_VAL(1, EPIC_VL_CFG_ALPHA_SEL_Msk, EPIC_VL_CFG_ALPHA_SEL_Pos)
                                 | MAKE_REG_VAL(alpha, EPIC_VL_CFG_ALPHA_Msk, EPIC_VL_CFG_ALPHA_Pos);
            }
        }
        else
        {
            Vlayer_x->CFG |= MAKE_REG_VAL(EPIC_LAYER_OPAQUE, EPIC_VL_CFG_ALPHA_Msk, EPIC_VL_CFG_ALPHA_Pos);
        }

        if (IS_Ax_COLOR_MODE(config->color_mode))
        {
            HAL_ASSERT(config->color_en);
#ifndef SF32LB55X
            Vlayer_x->FILL = MAKE_REG_VAL(0x0, EPIC_VL_FILL_BG_MODE_Msk, EPIC_VL_FILL_BG_MODE_Pos)
                             | MAKE_REG_VAL(config->color_r, EPIC_VL_FILL_BG_R_Msk, EPIC_VL_FILL_BG_R_Pos)
                             | MAKE_REG_VAL(config->color_g, EPIC_VL_FILL_BG_G_Msk, EPIC_VL_FILL_BG_G_Pos)
                             | MAKE_REG_VAL(config->color_b, EPIC_VL_FILL_BG_B_Msk, EPIC_VL_FILL_BG_B_Pos);
#endif /* SF32LB55X */
        }
        else if (config->color_en)
        {
            Vlayer_x->CFG |= EPIC_VL_CFG_FILTER_EN;
            Vlayer_x->FILTER = MAKE_REG_VAL(0x0, EPIC_VL_FILTER_FILTER_MASK_Msk, EPIC_VL_FILTER_FILTER_MASK_Pos)
                               | MAKE_REG_VAL(config->color_r, EPIC_VL_FILTER_FILTER_R_Msk, EPIC_VL_FILTER_FILTER_R_Pos)
                               | MAKE_REG_VAL(config->color_g, EPIC_VL_FILTER_FILTER_G_Msk, EPIC_VL_FILTER_FILTER_G_Pos)
                               | MAKE_REG_VAL(config->color_b, EPIC_VL_FILTER_FILTER_B_Msk, EPIC_VL_FILTER_FILTER_B_Pos);
        }


        Vlayer_x->TL_POS = MAKE_REG_VAL(config->x_offset, EPIC_VL_TL_POS_X0_Msk, EPIC_VL_TL_POS_X0_Pos)
                           | MAKE_REG_VAL(config->y_offset, EPIC_VL_TL_POS_Y0_Msk, EPIC_VL_TL_POS_Y0_Pos);
        Vlayer_x->BR_POS = MAKE_REG_VAL(config->x_offset + config->width - 1, EPIC_VL_BR_POS_X1_Msk, EPIC_VL_BR_POS_X1_Pos)
                           | MAKE_REG_VAL(config->y_offset + config->height - 1, EPIC_VL_BR_POS_Y1_Msk, EPIC_VL_BR_POS_Y1_Pos);
    }


    /*Setup transform registers*/
    Vlayer_x->EXTENTS = MAKE_REG_VAL(trans_result->clip_area.x1 - trans_result->clip_area.x0, EPIC_VL_EXTENTS_MAX_COL_Msk, EPIC_VL_EXTENTS_MAX_COL_Pos)
                        | MAKE_REG_VAL(trans_result->clip_area.y1 - trans_result->clip_area.y0, EPIC_VL_EXTENTS_MAX_LINE_Msk, EPIC_VL_EXTENTS_MAX_LINE_Pos);
    // clear
    Vlayer_x->ROT |= EPIC_VL_ROT_CALC_CLR;


    if (0 == trans_result->angle_degree)
    {
#if defined(SF32LB55X)||defined(SF32LB58X) //Use VL prefetch only after 58x
        if (line_bytes > EPIC_PREFETCH_THRESHOLD)  Vlayer_x->CFG |= EPIC_VL_CFG_PREFETCH_EN;
#endif /* SF32LB55X || SF32LB58X*/
        Vlayer_x_trans->ROT_M_CFG1 &= ~EPIC_VL_ROT_M_CFG1_M_MODE;
        Vlayer_x->MISC_CFG &= ~EPIC_VL_MISC_CFG_DEG_FORCE;
        MODIFY_REG(Vlayer_x->ROT, EPIC_VL_ROT_ROT_DEG_Msk, MAKE_REG_VAL(0, EPIC_VL_ROT_ROT_DEG_Msk, EPIC_VL_ROT_ROT_DEG_Pos));
    }
    else
    {

        MODIFY_REG(Vlayer_x->MISC_CFG, EPIC_VL_MISC_CFG_SIN_FORCE_VALUE_Msk,
                   MAKE_REG_VAL(trans_result->abs_sinma >> (EPIC_SIN_COS_FRAC_BIT - EPIC_VL_MISC_CFG_SIN_FRAC_BIT),
                                EPIC_VL_MISC_CFG_SIN_FORCE_VALUE_Msk,
                                EPIC_VL_MISC_CFG_SIN_FORCE_VALUE_Pos));
        MODIFY_REG(Vlayer_x->MISC_CFG, EPIC_VL_MISC_CFG_COS_FORCE_VALUE_Msk,
                   MAKE_REG_VAL(trans_result->abs_cosma >> (EPIC_SIN_COS_FRAC_BIT - EPIC_VL_MISC_CFG_COS_FRAC_BIT),
                                EPIC_VL_MISC_CFG_COS_FORCE_VALUE_Msk,
                                EPIC_VL_MISC_CFG_COS_FORCE_VALUE_Pos));
        Vlayer_x->MISC_CFG |= EPIC_VL_MISC_CFG_DEG_FORCE;

        MODIFY_REG(Vlayer_x->ROT, EPIC_VL_ROT_ROT_DEG_Msk, MAKE_REG_VAL(trans_result->angle_degree, EPIC_VL_ROT_ROT_DEG_Msk, EPIC_VL_ROT_ROT_DEG_Pos));

        /*VL_ROT_M_CFG1 is area size of rotated img, NOT include scale */
        Vlayer_x_trans->ROT_M_CFG1 = EPIC_VL_ROT_M_CFG1_M_MODE
                                     | MAKE_REG_VAL(trans_result->rotated_width, EPIC_VL_ROT_M_CFG1_M_ROT_MAX_COL_Msk, EPIC_VL_ROT_M_CFG1_M_ROT_MAX_COL_Pos)
                                     | MAKE_REG_VAL(trans_result->rotated_height, EPIC_VL_ROT_M_CFG1_M_ROT_MAX_LINE_Msk, EPIC_VL_ROT_M_CFG1_M_ROT_MAX_LINE_Pos);

        /* pivot coordinate is relative to video layer topleft point, for now it must be positive value  */
        Vlayer_x_trans->ROT_M_CFG2 = MAKE_REG_VAL(trans_result->pivot.x, EPIC_VL_ROT_M_CFG2_M_PIVOT_X_Msk, EPIC_VL_ROT_M_CFG2_M_PIVOT_X_Pos)
                                     | MAKE_REG_VAL(trans_result->pivot.y, EPIC_VL_ROT_M_CFG2_M_PIVOT_Y_Msk, EPIC_VL_ROT_M_CFG2_M_PIVOT_Y_Pos);

        /* set image position before rotation, its coordinate is relative video layer topleft point*/
        Vlayer_x_trans->ROT_M_CFG3 = MAKE_REG_VAL(trans_result->src_img.x, EPIC_VL_ROT_M_CFG3_M_XTL_Msk, EPIC_VL_ROT_M_CFG3_M_XTL_Pos)
                                     | MAKE_REG_VAL(trans_result->src_img.y, EPIC_VL_ROT_M_CFG3_M_YTL_Msk, EPIC_VL_ROT_M_CFG3_M_YTL_Pos);

    }


#ifndef SF32LB52X
    if (trans_result->h_mirror)
        Vlayer_x->MISC_CFG |= EPIC_VL_MISC_CFG_MIRROR;
    else
        Vlayer_x->MISC_CFG &= ~EPIC_VL_MISC_CFG_MIRROR;
#else
    if (trans_result->h_mirror)
        Vlayer_x->MISC_CFG |= EPIC_VL_MISC_CFG_H_MIRROR;
    else
        Vlayer_x->MISC_CFG &= ~EPIC_VL_MISC_CFG_H_MIRROR;

    if (trans_result->v_mirror)
        Vlayer_x->MISC_CFG |= EPIC_VL_MISC_CFG_V_MIRROR;
    else
        Vlayer_x->MISC_CFG &= ~EPIC_VL_MISC_CFG_V_MIRROR;
#endif /* SF32LB52X */

#ifndef SF32LB55X
    MODIFY_REG(Vlayer_x_trans->SCALE_INIT_CFG1, EPIC_VL_SCALE_INIT_CFG1_X_VAL_Msk,
               MAKE_REG_VAL(trans_result->scale_init_x, EPIC_VL_SCALE_INIT_CFG1_X_VAL_Msk, EPIC_VL_SCALE_INIT_CFG1_X_VAL_Pos));
    MODIFY_REG(Vlayer_x_trans->SCALE_INIT_CFG2, EPIC_VL_SCALE_INIT_CFG2_Y_VAL_Msk,
               MAKE_REG_VAL(trans_result->scale_init_y, EPIC_VL_SCALE_INIT_CFG2_Y_VAL_Msk, EPIC_VL_SCALE_INIT_CFG2_Y_VAL_Pos));


    if (EPIC_COLOR_L8 == config->color_mode)
    {
        uint32_t tab_id = EPIC_Allocate_L8Table(epic);

        HAL_ASSERT(tab_id <= (EPIC_VL_MISC_CFG_CLUT_SEL_Msk >> EPIC_VL_MISC_CFG_CLUT_SEL_Pos));
        EPIC_Overwrite_L8Table(epic_handle, tab_id, config->lookup_table, config->lookup_table_size);
        Vlayer_x->MISC_CFG &= ~EPIC_VL_MISC_CFG_CLUT_SEL_Msk;
        Vlayer_x->MISC_CFG |= MAKE_REG_VAL(tab_id, EPIC_VL_MISC_CFG_CLUT_SEL_Msk, EPIC_VL_MISC_CFG_CLUT_SEL_Pos);
    }

    Vlayer_x->SCALE_RATIO_H = MAKE_REG_VAL(trans_result->epic_scale_x, EPIC_VL_SCALE_RATIO_H_XPITCH_Msk, EPIC_VL_SCALE_RATIO_H_XPITCH_Pos);
    Vlayer_x->SCALE_RATIO_V = MAKE_REG_VAL(trans_result->epic_scale_y, EPIC_VL_SCALE_RATIO_V_YPITCH_Msk, EPIC_VL_SCALE_RATIO_V_YPITCH_Pos);
#else
    Vlayer_x->SCALE_RATIO = MAKE_REG_VAL(trans_result->epic_scale_x, EPIC_VL_SCALE_RATIO_XPITCH_Msk, EPIC_VL_SCALE_RATIO_XPITCH_Pos)
                            | MAKE_REG_VAL(trans_result->epic_scale_y, EPIC_VL_SCALE_RATIO_YPITCH_Msk, EPIC_VL_SCALE_RATIO_YPITCH_Pos);
#endif /* SF32LB55X */



    Vlayer_x->CFG |= MAKE_REG_VAL(layer_color_format, EPIC_VL_CFG_FORMAT_Msk, EPIC_VL_CFG_FORMAT_Pos)
                     | MAKE_REG_VAL(layer_depth, EPIC_VL_CFG_BLEND_DEPTH_Msk, EPIC_VL_CFG_BLEND_DEPTH_Pos);

    if (EPIC_COLOR_MONO != config->color_mode)
    {
        Vlayer_x->CFG |= MAKE_REG_VAL(line_bytes, EPIC_VL_CFG_WIDTH_Msk, EPIC_VL_CFG_WIDTH_Pos);
    }
    else
    {
        ;//Keep layer width as 0
    }

#if defined(SF32LB56X)||defined(SF32LB52X)
    if (IS_YUV_COLOR_MODE(config->color_mode))
    {
        Vlayer_x->CFG |= EPIC_VL_CFG_ACTIVE;
    }
    else
#endif
    {

        if (Vlayer_x->SRC != 0)
            Vlayer_x->CFG |= EPIC_VL_CFG_ACTIVE;
        else
            Vlayer_x->CFG &= ~EPIC_VL_CFG_ACTIVE;
    }


    return HAL_OK;
}


/**
 * @brief  Configure video layer in continues mode
 *
 * @param  epic EPIC instance
 * @param  idx - 0 - bg,  1 - fg,  2- mask
 * @param  config layer configuration
 *
 * @retval HAL status
 */
static HAL_StatusTypeDef EPIC_ContConfigVideoLayer(EPIC_HandleTypeDef *epic_handle, uint32_t idx,
        const EPIC_LayerConfigTypeDef *config)
{

    EPIC_VideoLayerxTypeDef *Vlayer_x;
    EPIC_TypeDef *epic = epic_handle->Instance;
    uint32_t vl_idx;

    int16_t x_offset = config->x_offset;
    int16_t y_offset = config->y_offset;

    //Check overflow
    if (EPCI_IS_TLBR_OVERFLOW(EPIC_VL, x_offset, y_offset, x_offset + config->width - 1, y_offset + config->height - 1))
    {
        //Overflow
        EPIC_DEBUG_PRINT_LAYER_INFO(config, "Overflow");
        HAL_ASSERT(0);
        return HAL_ERROR;
    }

    EPIC_LAYER_IDX layer_idx = epic_handle->api_cfg.cont_cfg.epic_layer[idx];
    Vlayer_x       = (EPIC_VideoLayerxTypeDef *)&epic->VL_CFG;


#ifndef SF32LB55X
    HAL_ASSERT((EPIC_LAYER_IDX_VL == layer_idx) || (EPIC_LAYER_IDX_2 == layer_idx));
    vl_idx = (EPIC_LAYER_IDX_VL == layer_idx) ? 0 : 1;
#else
    HAL_ASSERT(EPIC_LAYER_IDX_VL == layer_idx);
    vl_idx = 0;
#endif /* SF32LB55X */

    Vlayer_x += vl_idx;


#ifndef SF32LB55X
    Vlayer_x->SRC = HCPU_MPI_SBUS_ADDR(config->data);
#else
    Vlayer_x->SRC = (uint32_t)config->data;
#endif




    Vlayer_x->TL_POS = MAKE_REG_VAL(x_offset, EPIC_VL_TL_POS_X0_Msk, EPIC_VL_TL_POS_X0_Pos)
                       | MAKE_REG_VAL(y_offset, EPIC_VL_TL_POS_Y0_Msk, EPIC_VL_TL_POS_Y0_Pos);
    Vlayer_x->BR_POS = MAKE_REG_VAL(x_offset + config->width - 1, EPIC_VL_BR_POS_X1_Msk, EPIC_VL_BR_POS_X1_Pos)
                       | MAKE_REG_VAL(y_offset + config->height - 1, EPIC_VL_BR_POS_Y1_Msk, EPIC_VL_BR_POS_Y1_Pos);


    Vlayer_x->EXTENTS = MAKE_REG_VAL(config->width, EPIC_VL_EXTENTS_MAX_COL_Msk, EPIC_VL_EXTENTS_MAX_COL_Pos)
                        | MAKE_REG_VAL(config->height, EPIC_VL_EXTENTS_MAX_LINE_Msk, EPIC_VL_EXTENTS_MAX_LINE_Pos);

#ifndef SF32LB55X
    Vlayer_x->SCALE_RATIO_H = MAKE_REG_VAL(EPIC_SCALE_1, EPIC_VL_SCALE_RATIO_H_XPITCH_Msk, EPIC_VL_SCALE_RATIO_H_XPITCH_Pos);
    Vlayer_x->SCALE_RATIO_V = MAKE_REG_VAL(EPIC_SCALE_1, EPIC_VL_SCALE_RATIO_V_YPITCH_Msk, EPIC_VL_SCALE_RATIO_V_YPITCH_Pos);
#else
    Vlayer_x->SCALE_RATIO = MAKE_REG_VAL(EPIC_SCALE_1, EPIC_VL_SCALE_RATIO_XPITCH_Msk, EPIC_VL_SCALE_RATIO_XPITCH_Pos)
                            | MAKE_REG_VAL(EPIC_SCALE_1, EPIC_VL_SCALE_RATIO_YPITCH_Msk, EPIC_VL_SCALE_RATIO_YPITCH_Pos);
#endif /* SF32LB55X */

    if (IS_Ax_COLOR_MODE(config->color_mode))
    {
        HAL_ASSERT(config->color_en);
#ifndef SF32LB55X
        Vlayer_x->FILL = MAKE_REG_VAL(0x0, EPIC_VL_FILL_BG_MODE_Msk, EPIC_VL_FILL_BG_MODE_Pos)
                         | MAKE_REG_VAL(config->color_r, EPIC_VL_FILL_BG_R_Msk, EPIC_VL_FILL_BG_R_Pos)
                         | MAKE_REG_VAL(config->color_g, EPIC_VL_FILL_BG_G_Msk, EPIC_VL_FILL_BG_G_Pos)
                         | MAKE_REG_VAL(config->color_b, EPIC_VL_FILL_BG_B_Msk, EPIC_VL_FILL_BG_B_Pos);
#endif /* SF32LB55X */
    }

    uint32_t reg_v;

    uint32_t line_bytes = HAL_ALIGN(epic_handle->api_cfg.cont_cfg.color_depth[idx] * config->total_width, 8) >> 3;

    reg_v = MAKE_REG_VAL(line_bytes, EPIC_VL_CFG_WIDTH_Msk, EPIC_VL_CFG_WIDTH_Pos)
            | MAKE_REG_VAL(epic_handle->api_cfg.cont_cfg.color_format[idx], EPIC_VL_CFG_FORMAT_Msk, EPIC_VL_CFG_FORMAT_Pos)
            | MAKE_REG_VAL(1, EPIC_VL_CFG_BLEND_DEPTH_Msk, EPIC_VL_CFG_BLEND_DEPTH_Pos)
            | EPIC_VL_CFG_ACTIVE;

    if (!IS_NO_ALPHA_COLOR_MODE(config->color_mode))
    {
        reg_v |= MAKE_REG_VAL(1, EPIC_VL_CFG_ALPHA_BLEND_Msk, EPIC_VL_CFG_ALPHA_BLEND_Pos)
                 |  MAKE_REG_VAL(config->alpha, EPIC_VL_CFG_ALPHA_Msk, EPIC_VL_CFG_ALPHA_Pos);
    }
    else
    {
        reg_v |= MAKE_REG_VAL(1, EPIC_VL_CFG_ALPHA_SEL_Msk, EPIC_VL_CFG_ALPHA_SEL_Pos)
                 | MAKE_REG_VAL(config->alpha, EPIC_VL_CFG_ALPHA_Msk, EPIC_VL_CFG_ALPHA_Pos);
    }

    Vlayer_x->CFG = reg_v;
    return HAL_OK;
}


/**
 * @brief  Reset video layer transform settings in continues mode
 *
 * @param  epic EPIC instance
 * @param  idx - 0 - bg,  1 - fg,  2- mask
 * @param  config layer configuration
 *
 * @retval HAL status
 */
static HAL_StatusTypeDef EPIC_ContResetVideoLayer(EPIC_HandleTypeDef *epic_handle, uint32_t idx)
{

    EPIC_VideoLayerxTypeDef *Vlayer_x;
    EPIC_VideoLayerxTransTypeDef *Vlayer_x_trans;
    EPIC_TypeDef *epic = epic_handle->Instance;
    uint32_t vl_idx;


    EPIC_LAYER_IDX layer_idx = epic_handle->api_cfg.cont_cfg.epic_layer[idx];

    Vlayer_x       = (EPIC_VideoLayerxTypeDef *)&epic->VL_CFG;
    Vlayer_x_trans = (EPIC_VideoLayerxTransTypeDef *)&epic->VL_ROT_M_CFG1;


#ifndef SF32LB55X
    HAL_ASSERT((EPIC_LAYER_IDX_VL == layer_idx) || (EPIC_LAYER_IDX_2 == layer_idx));
    vl_idx = (EPIC_LAYER_IDX_VL == layer_idx) ? 0 : 1;
#else
    HAL_ASSERT(EPIC_LAYER_IDX_VL == layer_idx);
    vl_idx = 0;
#endif /* SF32LB55X */

    Vlayer_x += vl_idx;
    Vlayer_x_trans += vl_idx;



    Vlayer_x->ROT |= EPIC_VL_ROT_CALC_CLR;

    Vlayer_x_trans->ROT_M_CFG1 &= ~EPIC_VL_ROT_M_CFG1_M_MODE;
    Vlayer_x->MISC_CFG &= ~EPIC_VL_MISC_CFG_DEG_FORCE;
    MODIFY_REG(Vlayer_x->ROT, EPIC_VL_ROT_ROT_DEG_Msk, MAKE_REG_VAL(0, EPIC_VL_ROT_ROT_DEG_Msk, EPIC_VL_ROT_ROT_DEG_Pos));


#ifndef SF32LB52X
    Vlayer_x->MISC_CFG &= ~EPIC_VL_MISC_CFG_MIRROR;
#else
    Vlayer_x->MISC_CFG &= ~(EPIC_VL_MISC_CFG_H_MIRROR | EPIC_VL_MISC_CFG_V_MIRROR);
#endif /* SF32LB52X */

#ifndef SF32LB55X
    Vlayer_x_trans->SCALE_INIT_CFG1 = 0;
    Vlayer_x_trans->SCALE_INIT_CFG2 = 0;

    Vlayer_x->SCALE_RATIO_H = MAKE_REG_VAL(EPIC_SCALE_1, EPIC_VL_SCALE_RATIO_H_XPITCH_Msk, EPIC_VL_SCALE_RATIO_H_XPITCH_Pos);
    Vlayer_x->SCALE_RATIO_V = MAKE_REG_VAL(EPIC_SCALE_1, EPIC_VL_SCALE_RATIO_V_YPITCH_Msk, EPIC_VL_SCALE_RATIO_V_YPITCH_Pos);
#else
    Vlayer_x->SCALE_RATIO   = MAKE_REG_VAL(EPIC_SCALE_1, EPIC_VL_SCALE_RATIO_XPITCH_Msk, EPIC_VL_SCALE_RATIO_XPITCH_Pos)
                              | MAKE_REG_VAL(EPIC_SCALE_1, EPIC_VL_SCALE_RATIO_YPITCH_Msk, EPIC_VL_SCALE_RATIO_YPITCH_Pos);
#endif /* SF32LB55X */


    return HAL_OK;
}

#ifdef EPIC_SUPPORT_MASK

/**
 * @brief  Enable and configure mask layer
 *
 * @param  epic EPIC instance
 * @param  layer_idx The index of layer which will be masked
 * @param  config config
 *
 * @retval HAL status
 */
static HAL_StatusTypeDef EPIC_ConfigMaskLayer(EPIC_HandleTypeDef *hepic, EPIC_LAYER_IDX layer_idx,
        EPIC_BlendingDataType *config)
{
    uint32_t color_depth;
    uint32_t layer_color_format;
    uint32_t alpha_mix_mode;
    EPIC_TypeDef *epic = hepic->Instance;

    if ((config->x_offset < 0)
            || (config->y_offset < 0))
    {
        RETURN_ERROR(hepic, HAL_ERROR);
    }

    HAL_ASSERT(layer_idx < EPIC_LAYER_IDX_MAX);

    //Check overflow
    if (EPCI_IS_TLBR_OVERFLOW(EPIC_MASK, config->x_offset,
                              config->y_offset,
                              config->x_offset + config->width - 1,
                              config->y_offset + config->height - 1))
    {
        //Overflow
        HAL_ASSERT(0);
        return HAL_ERROR;
    }

    epic->MASK_TL_POS = MAKE_REG_VAL(config->x_offset, EPIC_MASK_TL_POS_X0_Msk, EPIC_MASK_TL_POS_X0_Pos)
                        | MAKE_REG_VAL(config->y_offset, EPIC_MASK_TL_POS_Y0_Msk, EPIC_MASK_TL_POS_Y0_Pos);
    epic->MASK_BR_POS = MAKE_REG_VAL(config->x_offset + config->width - 1, EPIC_MASK_BR_POS_X1_Msk, EPIC_MASK_BR_POS_X1_Pos)
                        | MAKE_REG_VAL(config->y_offset + config->height - 1, EPIC_MASK_BR_POS_Y1_Msk, EPIC_MASK_BR_POS_Y1_Pos);
    epic->MASK_SRC = (uint32_t)HCPU_MPI_SBUS_ADDR(config->data);

    color_depth = EPIC_GetColorDepth(config->color_mode);
    layer_color_format = EPIC_GetMaskLayerColorFormat(config->color_mode);

    if (ALPHA_BLEND_MASK == config->ax_mode)
    {
        alpha_mix_mode = EPIC_MASK_CFG_MIX_MODE_MULT;
    }
    else if (ALPHA_BLEND_OVERWRITE == config->ax_mode)
    {
        alpha_mix_mode = EPIC_MASK_CFG_MIX_MODE_OVERWRITE;
    }
    else
    {
        HAL_ASSERT(0);
        return HAL_ERROR;
    }

    uint32_t line_bytes = HAL_ALIGN(color_depth * config->total_width, 8) >> 3;
    uint32_t output_line_bytes = HAL_ALIGN(color_depth * config->width, 8) >> 3;

    epic->MASK_CFG = layer_color_format
                     | MAKE_REG_VAL(line_bytes, EPIC_MASK_CFG_WIDTH_Msk, EPIC_MASK_CFG_WIDTH_Pos)
                     | alpha_mix_mode
                     | (LayerIdx2MaskCh(layer_idx) << EPIC_MASK_CFG_L0_MASK_EN_Pos);

    if (output_line_bytes > EPIC_PREFETCH_THRESHOLD)
    {
        epic->MASK_CFG |= EPIC_MASK_CFG_PREFETCH_EN;
    }

    if (epic->MASK_SRC != 0)
        epic->MASK_CFG |= EPIC_MASK_CFG_ACTIVE;
    else
        epic->MASK_CFG &= ~EPIC_MASK_CFG_ACTIVE;

    return HAL_OK;
}

/**
 * @brief  Enable and configure mask layer in continue mode
 *
 * @param  epic EPIC instance
 * @param  layer_idx The index of layer which will be masked
 * @param  config config
 *
 * @retval HAL status
 */
static HAL_StatusTypeDef EPIC_ContConfigMaskLayer(EPIC_HandleTypeDef *hepic, uint32_t idx,
        EPIC_BlendingDataType *config)
{
    uint32_t color_depth;
    uint32_t layer_color_format;
    uint32_t alpha_mix_mode;
    EPIC_TypeDef *epic = hepic->Instance;

    int16_t x_offset = config->x_offset;
    int16_t y_offset = config->y_offset;

    if ((x_offset < 0)
            || (y_offset < 0))
    {
        RETURN_ERROR(hepic, HAL_ERROR);
    }

    EPIC_LAYER_IDX layer_idx = hepic->api_cfg.cont_cfg.epic_layer[idx];

    HAL_ASSERT(layer_idx < EPIC_LAYER_IDX_MAX);

    //Check overflow
    if (EPCI_IS_TLBR_OVERFLOW(EPIC_MASK, x_offset,
                              y_offset,
                              x_offset + config->width - 1,
                              y_offset + config->height - 1))
    {
        //Overflow
        HAL_ASSERT(0);
        return HAL_ERROR;
    }

    epic->MASK_TL_POS = MAKE_REG_VAL(x_offset, EPIC_MASK_TL_POS_X0_Msk, EPIC_MASK_TL_POS_X0_Pos)
                        | MAKE_REG_VAL(y_offset, EPIC_MASK_TL_POS_Y0_Msk, EPIC_MASK_TL_POS_Y0_Pos);
    epic->MASK_BR_POS = MAKE_REG_VAL(x_offset + config->width - 1, EPIC_MASK_BR_POS_X1_Msk, EPIC_MASK_BR_POS_X1_Pos)
                        | MAKE_REG_VAL(y_offset + config->height - 1, EPIC_MASK_BR_POS_Y1_Msk, EPIC_MASK_BR_POS_Y1_Pos);
    epic->MASK_SRC = (uint32_t)HCPU_MPI_SBUS_ADDR(config->data);

    color_depth = hepic->api_cfg.cont_cfg.color_depth[CONT_MASK_LAYER];
    layer_color_format = hepic->api_cfg.cont_cfg.color_format[CONT_MASK_LAYER];
    if ((layer_color_format != EPIC_MASK_CFG_FMT_A4) && (layer_color_format != EPIC_MASK_CFG_FMT_A8))
    {
        HAL_ASSERT(0);
        return HAL_ERROR;
    }

    if (ALPHA_BLEND_MASK == config->ax_mode)
    {
        alpha_mix_mode = EPIC_MASK_CFG_MIX_MODE_MULT;
    }
    else if (ALPHA_BLEND_OVERWRITE == config->ax_mode)
    {
        alpha_mix_mode = EPIC_MASK_CFG_MIX_MODE_OVERWRITE;
    }
    else
    {
        HAL_ASSERT(0);
        return HAL_ERROR;
    }

    uint32_t line_bytes = HAL_ALIGN(color_depth * config->total_width, 8) >> 3;
    uint32_t output_line_bytes = HAL_ALIGN(color_depth * config->width, 8) >> 3;

    epic->MASK_CFG = layer_color_format
                     | MAKE_REG_VAL(line_bytes, EPIC_MASK_CFG_WIDTH_Msk, EPIC_MASK_CFG_WIDTH_Pos)
                     | alpha_mix_mode
                     | (LayerIdx2MaskCh(layer_idx) << EPIC_MASK_CFG_L0_MASK_EN_Pos);

    if (output_line_bytes > EPIC_PREFETCH_THRESHOLD)
    {
        epic->MASK_CFG |= EPIC_MASK_CFG_PREFETCH_EN;
    }

    if (epic->MASK_SRC != 0)
        epic->MASK_CFG |= EPIC_MASK_CFG_ACTIVE;
    else
        epic->MASK_CFG &= ~EPIC_MASK_CFG_ACTIVE;

    return HAL_OK;
}


/**
 * @brief  Disable mask layer
 *
 * @param  epic EPIC instance
 *
 * @retval HAL status
 */
static HAL_StatusTypeDef EPIC_DisableMaskLayer(EPIC_TypeDef *epic)
{
    MODIFY_REG(epic->MASK_CFG, EPIC_MASK_CFG_ACTIVE_Msk, 0);

    return HAL_OK;
}
#else
static HAL_StatusTypeDef EPIC_ConfigMaskLayer(EPIC_HandleTypeDef *hepic, EPIC_LAYER_IDX layer_idx,
        EPIC_BlendingDataType *config)
{
    RETURN_ERROR(hepic, HAL_ERROR);
}

static HAL_StatusTypeDef EPIC_ContConfigMaskLayer(EPIC_HandleTypeDef *hepic, uint32_t idx,
        EPIC_BlendingDataType *config)
{
    RETURN_ERROR(hepic, HAL_ERROR);
}

static HAL_StatusTypeDef EPIC_DisableMaskLayer(EPIC_TypeDef *epic)
{
    return HAL_ERROR;
}
#endif /* SF32LB55X */


#ifdef HAL_EZIP_MODULE_ENABLED
static HAL_StatusTypeDef EPIC_ConfigEzipDec(EPIC_HandleTypeDef *epic,
        EPIC_BlendingDataType *ezip_layer,
        EPIC_BlendingDataType *dst,
        EPIC_TransformResultDef *trans_result)
{
    EZIP_TypeDef *ezip;
    EPIC_AreaTypeDef intersect_area;
    bool is_intersect;
    uint16_t start_col;
    uint16_t end_col;
    uint16_t start_row;
    uint16_t end_row;
    EZIP_DecodeConfigTypeDef config;
    HAL_StatusTypeDef res = HAL_OK;
    uint32_t scale_x;
    uint32_t scale_y;
    uint32_t scale_init_x;
    uint32_t scale_init_y;
    uint16_t old_width;
    uint16_t old_height;


    scale_x = trans_result->epic_scale_x;
    scale_y = trans_result->epic_scale_y;
    scale_init_x = trans_result->scale_init_x;
    scale_init_y = trans_result->scale_init_y;

    old_width = trans_result->width;
    old_height = trans_result->height;

    HAL_ASSERT(old_width > 0);
    ezip = epic->hezip->Instance;
    HAL_ASSERT(ezip_layer);

    is_intersect = EPIC_CalcIntersectArea(ezip_layer, dst, &intersect_area);
    if (is_intersect)
    {
        bool log_en = true;
        start_col = intersect_area.x0 - ezip_layer->x_offset;
        end_col = intersect_area.x1 - ezip_layer->x_offset;

        start_row = intersect_area.y0 - ezip_layer->y_offset;
        end_row = intersect_area.y1 - ezip_layer->y_offset;
#if 1
        if (false || (0 != start_col) || (0 != start_row)
                || (ezip_layer->width != (end_col + 1))
                || (ezip_layer->height != (end_row + 1)))
        {
            log_en = true;
            EPIC_PRINTF("area:%d,%d,%d,%d;%d,%d\n", intersect_area.x0, intersect_area.y0,
                        intersect_area.x1, intersect_area.y1, scale_x, scale_y);
            EPIC_PRINTF("dst:%d,%d,%d,%d\n", dst->x_offset, dst->y_offset,
                        dst->x_offset + dst->width - 1, dst->y_offset + dst->height - 1);
            EPIC_PRINTF("ezip:%d,%d,%d,%d\n", ezip_layer->x_offset, ezip_layer->y_offset,
                        ezip_layer->x_offset + ezip_layer->width - 1, ezip_layer->y_offset + ezip_layer->height - 1);
            EPIC_PRINTF("old:%d,%d,%d,%d\n", start_col, start_row, end_col, end_row);
            EPIC_PRINTF("old_w:%d,old_h:%d\n", old_width, old_height);
        }
#endif
        /* update area if image is zoomed, otherwise ezip would produced more data than epic required */
        if ((scale_x != EPIC_SCALE_1) || (scale_init_x != 0))
        {
            start_col = (start_col * scale_x + scale_init_x) >> EPIC_SCALE_FACTOR_FRAC_SIZE;
            /* add one column, such that epic can always get sufficient data */
            end_col = (end_col * scale_x + scale_init_x + EPIC_SCALE_1) >> EPIC_SCALE_FACTOR_FRAC_SIZE;
        }
        start_col += trans_result->clip_area.x0;
        end_col   +=  trans_result->clip_area.x0;
        /* end_col cannot exceed the last column,
           otherwise epic would get wrong number of columns from ezip */

        if (end_col > trans_result->clip_area.x1)
        {
            end_col = trans_result->clip_area.x1;
        }

        if ((scale_y != EPIC_SCALE_1) || (scale_init_y != 0))
        {
            start_row = (start_row * scale_y + scale_init_y) >> EPIC_SCALE_FACTOR_FRAC_SIZE;
            /* add one row, such that epic can always get sufficient data */
            end_row = ((end_row * scale_y + scale_init_y + EPIC_SCALE_1) >> EPIC_SCALE_FACTOR_FRAC_SIZE);
        }
        start_row +=  trans_result->clip_area.y0;
        end_row   +=  trans_result->clip_area.y0;

        if (end_row > trans_result->clip_area.y1)
        {
            end_row = trans_result->clip_area.y1;
        }


#if 1
        if (log_en)
        {
            EPIC_PRINTF("new:%d,%d,%d,%d\n", start_col, start_row, end_col, end_row);
        }
#endif

        config.input = (uint8_t *)ezip_layer->data;
        config.output = NULL;
        config.work_mode = HAL_EZIP_MODE_EZIP;
        config.output_mode = HAL_EZIP_OUTPUT_EPIC;
        config.start_x = start_col;
        config.start_y = start_row;
        config.width = end_col - start_col + 1;
        config.height = end_row - start_row + 1;
        config.input_data_size = ezip_layer->data_size;
        EPIC_PRINTF("ezip_config_dat:%x x:%d y: %d w:%d h:%d\n", config.input, config.start_x, config.start_y, config.width, config.height);
        epic->hezip->CpltCallback = EPIC_EzipCpltCallback;
        epic->hezip->user_data = (void *)epic;
        res = HAL_EZIP_Decode_IT(epic->hezip, &config);
        //RT_ASSERT(res == HAL_OK);
        HAL_ASSERT(res == HAL_OK);
    }

    return res;
}



#endif /* HAL_EZIP_MODULE_ENABLED */

#ifdef EPIC_DEBUG
static inline EPIC_OpHistItemTypeDef *EPIC_AllocRecordItem(EPIC_HandleTypeDef *epic)
{
    EPIC_OpHistItemTypeDef *p;

    if (!epic->op_hist)
    {
        return NULL;
    }

    epic->op_hist->idx = (epic->op_hist->idx + 1) & (EPIC_OP_HIST_DEPTH - 1);
    p = &epic->op_hist->hist[epic->op_hist->idx];

    p->start_time = HAL_HPAON_READ_GTIMER();
    return p;
}
static void EPIC_RecordRotationOp(EPIC_HandleTypeDef *epic, EPIC_TransformCfgTypeDef *rot_cfg,
                                  EPIC_BlendingDataType *fg, EPIC_BlendingDataType *bg,
                                  EPIC_BlendingDataType *dst, uint8_t alpha)
{
    EPIC_OpHistItemTypeDef *item;

    item = EPIC_AllocRecordItem(epic);
    if (!item)
    {
        return;
    }

    item->op = EPIC_OP_ROTATION;
    memcpy((void *)&item->param.rot_param.rot_cfg, (void *)rot_cfg, sizeof(item->param.rot_param.rot_cfg));
    memcpy((void *)&item->param.rot_param.fg, (void *)fg, sizeof(item->param.rot_param.fg));
    memcpy((void *)&item->param.rot_param.bg, (void *)bg, sizeof(item->param.rot_param.bg));
    memcpy((void *)&item->param.rot_param.dst, (void *)dst, sizeof(item->param.rot_param.dst));
    item->param.rot_param.alpha = alpha;

}

static void EPIC_RecordBlendingOp(EPIC_HandleTypeDef *epic,
                                  EPIC_BlendingDataType *fg, EPIC_BlendingDataType *bg,
                                  EPIC_BlendingDataType *dst, uint8_t alpha)
{
    EPIC_OpHistItemTypeDef *item;

    item = EPIC_AllocRecordItem(epic);
    if (!item)
    {
        return;
    }

    item->op = EPIC_OP_BLENDING;
    memcpy((void *)&item->param.blend_param.fg, (void *)fg, sizeof(item->param.blend_param.fg));
    memcpy((void *)&item->param.blend_param.bg, (void *)bg, sizeof(item->param.blend_param.bg));
    memcpy((void *)&item->param.blend_param.dst, (void *)dst, sizeof(item->param.blend_param.dst));
    item->param.blend_param.alpha = alpha;
}

static void EPIC_RecordFillingOp(EPIC_HandleTypeDef *epic, EPIC_FillingCfgTypeDef *param)
{
    EPIC_OpHistItemTypeDef *item;

    item = EPIC_AllocRecordItem(epic);
    if (!item)
    {
        return;
    }

    item->op = EPIC_OP_FILLING;
    memcpy(&item->param.fill_param, param, sizeof(item->param.fill_param));
}

static void EPIC_RecordGradOp(EPIC_HandleTypeDef *epic, EPIC_GradCfgTypeDef *param)
{
    EPIC_OpHistItemTypeDef *item;

    item = EPIC_AllocRecordItem(epic);
    if (!item)
    {
        return;
    }

    item->op = EPIC_OP_FILLING;
    memcpy(&item->param.grad_param, param, sizeof(item->param.grad_param));
}

static void EPIC_RecordBlendingExOp(EPIC_HandleTypeDef *epic,
                                    EPIC_LayerConfigTypeDef *input_layer,
                                    uint8_t input_layer_num,
                                    EPIC_LayerConfigTypeDef *output_layer)
{
    EPIC_OpHistItemTypeDef *item;

    item = EPIC_AllocRecordItem(epic);
    if (!item)
    {
        return;
    }

    item->op = EPIC_OP_BLENDING_EX;

    memcpy((void *)item->param.blend_ex_param.input_layer, (void *)input_layer,  sizeof(EPIC_LayerConfigTypeDef) * input_layer_num);
    memcpy((void *)&item->param.blend_ex_param.output_layer, (void *)output_layer, sizeof(EPIC_LayerConfigTypeDef));
    item->param.blend_ex_param.input_layer_num = input_layer_num;
}

static void EPIC_RecordCopyOp(EPIC_HandleTypeDef *epic,
                              EPIC_BlendingDataType *src,
                              EPIC_BlendingDataType *dst)
{
    EPIC_OpHistItemTypeDef *item;

    item = EPIC_AllocRecordItem(epic);
    if (!item)
    {
        return;
    }

    item->op = EPIC_OP_COPY;
    memcpy((void *)&item->param.copy_param.src, (void *)src, sizeof(item->param.copy_param.src));
    memcpy((void *)&item->param.copy_param.dst, (void *)dst, sizeof(item->param.copy_param.dst));
}

static void EPIC_RecordContBlendingOp(EPIC_HandleTypeDef *epic,
                                      EPIC_LayerConfigTypeDef *input_layer,
                                      EPIC_LayerConfigTypeDef *mask_layer,
                                      EPIC_LayerConfigTypeDef *output_layer)
{
    EPIC_OpHistItemTypeDef *item;

    item = EPIC_AllocRecordItem(epic);
    if (!item)
    {
        return;
    }

    item->op = EPIC_OP_CONT_BLENDING;

    memcpy((void *)&item->param.cont_blend_repeat_param.input_layer, (void *)input_layer,  sizeof(EPIC_LayerConfigTypeDef));
    if (mask_layer)
        memcpy((void *)&item->param.cont_blend_repeat_param.mask_layer, (void *)mask_layer,   sizeof(EPIC_LayerConfigTypeDef));
    else
        memset((void *)&item->param.cont_blend_repeat_param.mask_layer,    0,   sizeof(EPIC_LayerConfigTypeDef));

    memcpy((void *)&item->param.cont_blend_repeat_param.output_layer, (void *)output_layer, sizeof(EPIC_LayerConfigTypeDef));
}

#ifdef EPIC_CONT_BLEND_DEBUG
static void EPIC_RecordContBlendingRepeatOp(EPIC_HandleTypeDef *epic,
        EPIC_LayerConfigTypeDef *input_layer,
        EPIC_LayerConfigTypeDef *mask_layer,
        EPIC_LayerConfigTypeDef *output_layer)
{
    EPIC_OpHistItemTypeDef *item;

    item = EPIC_AllocRecordItem(epic);
    if (!item)
    {
        return;
    }

    item->op = EPIC_OP_CONT_BLENDING_REPEAT;

    memcpy((void *)&item->param.cont_blend_repeat_param.input_layer, (void *)input_layer,  sizeof(EPIC_LayerConfigTypeDef));
    if (mask_layer)
        memcpy((void *)&item->param.cont_blend_repeat_param.mask_layer, (void *)mask_layer,   sizeof(EPIC_LayerConfigTypeDef));
    else
        memset((void *)&item->param.cont_blend_repeat_param.mask_layer,    0,   sizeof(EPIC_LayerConfigTypeDef));

    memcpy((void *)&item->param.cont_blend_repeat_param.output_layer, (void *)output_layer, sizeof(EPIC_LayerConfigTypeDef));
}
#endif /* EPIC_CONT_BLEND_DEBUG */

static void EPIC_RecordTransformOp(EPIC_HandleTypeDef *epic,
                                   EPIC_LayerConfigTypeDef *input_layer,
                                   uint8_t input_layer_num,
                                   EPIC_LayerConfigTypeDef *output_layer,
                                   EPIC_TransPath hor_path,
                                   EPIC_TransPath ver_path,
                                   void *user_data)
{
    EPIC_OpHistItemTypeDef *item;

    item = EPIC_AllocRecordItem(epic);
    if (!item)
    {
        return;
    }

    item->op = EPIC_OP_TRANSFORM;

    memcpy((void *)item->param.transform_param.input_layer, (void *)input_layer,  sizeof(EPIC_LayerConfigTypeDef) * input_layer_num);
    memcpy((void *)&item->param.transform_param.output_layer, (void *)output_layer, sizeof(EPIC_LayerConfigTypeDef));
    item->param.transform_param.input_layer_num = input_layer_num;
    item->param.transform_param.hor_path = hor_path;
    item->param.transform_param.ver_path = ver_path;
    item->param.transform_param.user_data = user_data;
}

#endif /* EPIC_DEBUG */

static void EPIC_CopyParam(EPIC_BlendingDataType *blending_data, EPIC_LayerConfigTypeDef *layer_config)
{
    HAL_ASSERT(blending_data && layer_config);

    memcpy(blending_data, layer_config, sizeof(EPIC_BlendingDataType));
}

static void EPIC_TransResultInit(EPIC_TransformResultDef *trans_result, const EPIC_BlendingDataType *layer)
{
    HAL_ASSERT(trans_result);
    HAL_ASSERT(layer);

    memset(trans_result, 0, sizeof(EPIC_TransformResultDef));

    trans_result->epic_scale_x = EPIC_SCALE_1;
    trans_result->epic_scale_y = EPIC_SCALE_1;

    trans_result->abs_sinma = 0 << EPIC_SIN_COS_FRAC_BIT;
    trans_result->abs_cosma = 1 << EPIC_SIN_COS_FRAC_BIT;


    trans_result->width  = layer->width;
    trans_result->height = layer->height;


    trans_result->clip_area.x0 = 0;
    trans_result->clip_area.y0 = 0;
    trans_result->clip_area.x1 = layer->width - 1;
    trans_result->clip_area.y1 = layer->height - 1;

}

#ifdef EPIC_SUPPORT_DITHER
static void EPIC_ConfigDither(EPIC_HandleTypeDef *hepic, uint32_t dither_mask)
{
    EPIC_TypeDef *epic = hepic->Instance;

    if ((0 == (dither_mask & 0x00FFFFFF)) || (0 == hepic->dither_enable))
    {
        epic->DITHER_CONF &= ~ EPIC_DITHER_CONF_EN;
    }
    else
    {
        uint32_t r_dither = (dither_mask >> 16) & 0xFF;
        uint32_t g_dither = (dither_mask >> 8) & 0xFF;
        uint32_t b_dither = (dither_mask >> 0) & 0xFF;

        epic->DITHER_CONF = EPIC_DITHER_CONF_EN  | EPIC_DITHER_CONF_LFSR_LOAD | EPIC_DITHER_CONF_LFSR_LOAD_SEL_Msk
                            | MAKE_REG_VAL(r_dither, EPIC_DITHER_CONF_W_R_Msk, EPIC_DITHER_CONF_W_R_Pos)
                            | MAKE_REG_VAL(g_dither, EPIC_DITHER_CONF_W_G_Msk, EPIC_DITHER_CONF_W_G_Pos)
                            | MAKE_REG_VAL(b_dither, EPIC_DITHER_CONF_W_B_Msk, EPIC_DITHER_CONF_W_B_Pos)
                            ;
    }
}
#endif /* EPIC_SUPPORT_DITHER */

static void EPIC_WaitDone(EPIC_HandleTypeDef *hepic)
{
#if defined(SF32LB55X)||defined(SF32LB58X)
    /*Wait previous blending done.*/
    while (hepic->Instance->STATUS != 0x0);

#else
    uint32_t cur_cnt, prev_cnt = hepic->Instance->PERF_CNT;
    /*Wait previous blending done.*/
    while (hepic->Instance->STATUS != 0x0);
    cur_cnt = hepic->Instance->PERF_CNT;

    hepic->WaitCnt += cur_cnt - prev_cnt;
    hepic->PerfCnt += cur_cnt;
    hepic->Instance->PERF_CNT = 0;
#endif
}

static void EPIC_InitRamInstance(EPIC_HandleTypeDef *hepic)
{
    if (hepic->RamInstance)
    {
        memset(hepic->RamInstance, 0, sizeof(EPIC_TypeDef));
#ifdef EPIC_SUPPORT_YUV
        hepic->RamInstance->COEF0 = 0x12A19204;
        hepic->RamInstance->COEF1 = 0x000664D0;
#endif /* EPIC_SUPPORT_YUV */
    }
}
static void EPIC_WaitValidInstance(EPIC_HandleTypeDef *hepic)
{
    if ((hepic->Instance->STATUS != 0x0) && (hepic->RamInstance) && (!hepic->RamInstance_used))
    {
        hepic->Instance = hepic->RamInstance;
        hepic->RamInstance_used = 1;
    }
    else
    {
        EPIC_WaitDone(hepic);
    }
}

static void EPIC_CommitInstance(EPIC_HandleTypeDef *hepic)
{
    if ((hepic->RamInstance_used) && (hepic->RamInstance))
    {
        hepic->RamInstance_used = 0;
        hepic->Instance = hepic->HwInstance;
        EPIC_WaitDone(hepic);
#ifdef SF32LB55X
        memcpy((void *)&hepic->Instance->CANVAS_TL_POS, (const void *)&hepic->RamInstance->CANVAS_TL_POS,
               ((uint32_t)&hepic->Instance->VL_ROT_M_CFG3) - ((uint32_t)&hepic->Instance->CANVAS_TL_POS) + 1);

#elif defined(SF32LB58X)
        memcpy((void *)&hepic->Instance->CANVAS_TL_POS, (const void *)&hepic->RamInstance->CANVAS_TL_POS,
               ((uint32_t)&hepic->Instance->RSVD1[0]) - ((uint32_t)&hepic->Instance->CANVAS_TL_POS));
#else
        memcpy((void *)&hepic->Instance->CANVAS_TL_POS, (const void *)&hepic->RamInstance->CANVAS_TL_POS,
               ((uint32_t)&hepic->Instance->PERF_CNT) - ((uint32_t)&hepic->Instance->CANVAS_TL_POS));
#endif /* SF32LB55X */

#ifdef EPIC_SUPPORT_L8
        if (hepic->RamLTabSize[0])
        {
            memcpy((void *)hepic->LTab[0], (void *)hepic->RamLTab[0],
                   ((uint32_t)hepic->RamLTabSize[0]) << 2);


            hepic->RamLTabSize[0] = 0;
        }

#if (EPIC_LOOKUP_TABLES > 1)
        if (hepic->RamLTabSize[1])
        {
            memcpy((void *)hepic->LTab[1], (void *)hepic->RamLTab[1],
                   ((uint32_t)hepic->RamLTabSize[1]) << 2);

            hepic->RamLTabSize[1] = 0;
        }
#endif /* EPIC_LOOKUP_TABLES > 1 */
#endif /* EPIC_SUPPORT_L8 */

    }
}

static void EPIC_DisposeRamInstance(EPIC_HandleTypeDef *hepic)
{
    if (hepic->RamInstance_used)
    {
        hepic->RamInstance_used = 0;
        hepic->Instance = hepic->HwInstance;
    }
}

HAL_StatusTypeDef HAL_EPIC_Init(EPIC_HandleTypeDef *epic)
{
    EPIC_InitTypeDef *init;

#ifdef HAL_EZIP_MODULE_ENABLED
    epic->hezip->user_data = (void *)epic;
#endif

    init = &epic->Init;


#ifdef EPIC_DEBUG
    if (epic->op_hist)
    {
        epic->op_hist->idx = 0;
    }
#endif
    epic->api_type = EPIC_API_NORMAL;
    epic->RamInstance_used = 0;
    epic->HwInstance = epic->Instance;
#ifdef EPIC_SUPPORT_L8
    epic->LTab[0] = (__IO uint32_t *)(((uint32_t)(epic->HwInstance)) + 0x400 * 1);
#if (EPIC_LOOKUP_TABLES > 1)
    epic->LTab[1] = (__IO uint32_t *)(((uint32_t)(epic->HwInstance)) + 0x400 * 2);
#endif /* EPIC_LOOKUP_TABLES */
    for (uint32_t i = 0; i < EPIC_LOOKUP_TABLES; i++)
    {
        epic->RamLTab[i] = NULL;
        epic->RamLTabSize[i] = 0;
    }
#endif /* EPIC_SUPPORT_L8 */
    EPIC_InitRamInstance(epic);

#ifndef SF32LB55X
    HAL_RCC_EnableModule(RCC_MOD_EPIC);
#endif /* SF32LB55X */

    epic->State = HAL_EPIC_STATE_READY;

    return HAL_OK;
}

/**
* @brief  Init an epic rotate cfg struct to default vaules
*
* @param[layer]  EPIC layer
* @retval None
*/
void HAL_EPIC_RotDataInit(EPIC_TransformCfgTypeDef *rot_cfg)
{
    HAL_ASSERT(NULL != rot_cfg);
    memset(rot_cfg, 0, sizeof(EPIC_TransformCfgTypeDef));
    rot_cfg->scale_x  = EPIC_INPUT_SCALING_FACTOR_1;
    rot_cfg->scale_y  = EPIC_INPUT_SCALING_FACTOR_1;
}

/**
 * @brief  Init an epic blend struct to default values
 *
 * @param layer  EPIC layer
 * @retval None
 */
void HAL_EPIC_BlendDataInit(EPIC_BlendingDataType *layer)
{
    HAL_ASSERT(NULL != layer);
    memset(layer, 0, sizeof(EPIC_BlendingDataType));
    layer->lookup_table_size = EPIC_MAX_LOOKUP_TABLE_CNT;
}

/**
 * @brief  Init an epic layer config struct to default values
 *
 * @param layer EPIC layer
 * @retval None
 */
void HAL_EPIC_LayerConfigInit(EPIC_LayerConfigTypeDef *layer)
{
    HAL_ASSERT(NULL != layer);
    memset(layer, 0, sizeof(*layer));
    layer->alpha = EPIC_LAYER_OPAQUE;
    layer->lookup_table_size = EPIC_MAX_LOOKUP_TABLE_CNT;

    HAL_EPIC_RotDataInit(&(layer->transform_cfg));
}


/**
 * @brief  Start blending in polling mode
 *
 * @param  epic EPIC instance
 * @param  fg foreground layer configuration
 * @param  bg background layer configuration
 * @param  dst output layer configuration
 * @param  alpha alpha value
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EPIC_BlendStart(EPIC_HandleTypeDef *epic,
                                      EPIC_BlendingDataType *fg,
                                      EPIC_BlendingDataType *bg,
                                      EPIC_BlendingDataType *dst,
                                      uint8_t alpha)
{
    EPIC_BlendingDataType bg_bak;
    EPIC_BlendingDataType fg_bak;
    EPIC_BlendingDataType dst_bak;

    HAL_StatusTypeDef ret = HAL_OK;


#ifdef SF32LB52X
    EPIC_TransformCfgTypeDef rot_cfg;

    HAL_EPIC_RotDataInit(&rot_cfg);

    return HAL_EPIC_Rotate(epic, &rot_cfg, fg, bg, dst, alpha);

#endif /* SF32LB52X */


    if (HAL_EPIC_STATE_READY != epic->State)
    {
        RETURN_ERROR(epic, HAL_BUSY);
    }
#ifndef SF32LB55X
    if (IS_ALPHA_MASK_LAYER(fg) && IS_ALPHA_MASK_LAYER(bg))
        RETURN_ERROR(epic, HAL_ERROR);//Only one mask layer supported.
#endif /* SF32LB55X */

    if (!EPIC_SUPPROT_OUT_FORMAT(dst->color_mode)) RETURN_ERROR(epic, HAL_ERROR);

    EPIC_ENABLE(epic);

#ifdef EPIC_DEBUG
    EPIC_RecordBlendingOp(epic, fg, bg, dst, alpha);
#endif

    epic->State = HAL_EPIC_STATE_BUSY;

    memcpy(&bg_bak,  bg,  sizeof(bg_bak));
    memcpy(&fg_bak,  fg,  sizeof(fg_bak));
    memcpy(&dst_bak, dst, sizeof(dst_bak));
    EPIC_MakeAllLayerCoordValid(&fg_bak, &bg_bak, &dst_bak);

    ret = EPIC_ConfigBlend(epic, &bg_bak, &fg_bak, &dst_bak, alpha);
    if (HAL_OK != ret)
    {
        ret = HAL_OK;
        goto __EXIT;
    }

    epic->Instance->COMMAND |= EPIC_COMMAND_START;
    EPIC_WaitDone(epic);
#ifdef HAL_EZIP_MODULE_ENABLED
    while (HAL_OK != HAL_EZIP_CheckReady(epic->hezip))
    {
        // wait for ezip idle
    }
#endif

__EXIT:

    EPIC_DisableLayer(epic->Instance, EPIC_LAYER_IDX_0);
    EPIC_DisableLayer(epic->Instance, EPIC_LAYER_IDX_1);
    EPIC_DisableVideoLayer(epic->Instance);
    EPIC_DisableMaskLayer(epic->Instance);
    epic->Instance->CANVAS_BG = 0;

    EPIC_DISABLE(epic);

    epic->State = HAL_EPIC_STATE_READY;

    return ret;
}

/**
 * @brief  Start blending in interrupt mode
 *
 * @param  epic EPIC instance
 * @param  fg foreground layer configuration
 * @param  bg background layer configuration
 * @param  dst output layer configuration
 * @param  alpha alpha value
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EPIC_BlendStart_IT(EPIC_HandleTypeDef *epic,
        EPIC_BlendingDataType *fg,
        EPIC_BlendingDataType *bg,
        EPIC_BlendingDataType *dst,
        uint8_t alpha)
{
    HAL_StatusTypeDef ret;
    EPIC_BlendingDataType bg_bak;
    EPIC_BlendingDataType fg_bak;
    EPIC_BlendingDataType dst_bak;

#ifdef SF32LB52X
    EPIC_TransformCfgTypeDef rot_cfg;

    HAL_EPIC_RotDataInit(&rot_cfg);

    return HAL_EPIC_Rotate_IT(epic, &rot_cfg, fg, bg, dst, alpha);

#endif /* SF32LB52X */

    HAL_ASSERT(NULL == epic->IntXferCpltCallback);

    if (HAL_EPIC_STATE_READY != epic->State)
    {
        RETURN_ERROR(epic, HAL_BUSY);
    }

#ifndef SF32LB55X
    if (IS_ALPHA_MASK_LAYER(fg) && IS_ALPHA_MASK_LAYER(bg))
        RETURN_ERROR(epic, HAL_ERROR);//Only one mask layer supported.
#endif /* SF32LB55X */

    if ((fg->width > fg->total_width) || (bg->width > bg->total_width))
    {
        RETURN_ERROR(epic, HAL_ERROR);
    }

    if (!EPIC_SUPPROT_OUT_FORMAT(dst->color_mode)) RETURN_ERROR(epic, HAL_ERROR);

    EPIC_ENABLE(epic);;

#ifdef EPIC_DEBUG
    EPIC_RecordBlendingOp(epic, fg, bg, dst, alpha);
#endif


    epic->State = HAL_EPIC_STATE_BUSY;
    memcpy(&bg_bak,  bg,  sizeof(bg_bak));
    memcpy(&fg_bak,  fg,  sizeof(fg_bak));
    memcpy(&dst_bak, dst, sizeof(dst_bak));
    EPIC_MakeAllLayerCoordValid(&fg_bak, &bg_bak, &dst_bak);

    ret = EPIC_ConfigBlend(epic, &bg_bak, &fg_bak, &dst_bak, alpha);
    if (HAL_OK != ret)
    {
        EPIC_BlendCpltCallback(epic);
        return HAL_OK;
    }

    epic->IntXferCpltCallback = EPIC_BlendCpltCallback;

    /* clear pending interrupt */
    epic->Instance->EOF_IRQ = epic->Instance->EOF_IRQ;
    /* unmask */
    epic->Instance->SETTING |= EPIC_SETTING_EOF_IRQ_MASK;
    /* trigger epic */
    epic->Instance->COMMAND |= EPIC_COMMAND_START;

    return HAL_OK;
}


HAL_StatusTypeDef HAL_EPIC_BlendStartEx(EPIC_HandleTypeDef *epic,
                                        EPIC_LayerConfigTypeDef *input_layer,
                                        uint8_t input_layer_num,
                                        EPIC_LayerConfigTypeDef *output_layer)
{

    HAL_StatusTypeDef ret;
    EPIC_BlendingDataType layer[MAX_EPIC_LAYER + 1];  /*+1 for output_layer*/
    uint8_t alpha[MAX_EPIC_LAYER];
    EPIC_TransformCfgTypeDef transform_cfg[MAX_EPIC_LAYER];
    uint32_t i;

    HAL_ASSERT(NULL == epic->IntXferCpltCallback);


    if (HAL_EPIC_STATE_READY != epic->State)
    {
        RETURN_ERROR(epic, HAL_BUSY);
    }

    if ((input_layer_num > MAX_EPIC_LAYER) || (0 == input_layer_num))
    {
        RETURN_ERROR(epic, HAL_ERROR);
    }

    for (i = 0; i < input_layer_num; i++)
    {
        if (input_layer[i].width > input_layer[i].total_width)
        {
            RETURN_ERROR(epic, HAL_ERROR);
        }
    }
    if (!EPIC_SUPPROT_OUT_FORMAT(output_layer->color_mode)) RETURN_ERROR(epic, HAL_ERROR);

    EPIC_ENABLE(epic);

#ifdef EPIC_DEBUG
    EPIC_RecordBlendingExOp(epic, input_layer, input_layer_num, output_layer);
#endif

    epic->State = HAL_EPIC_STATE_BUSY;

    for (i = 0; i < input_layer_num; i++)
    {
        EPIC_CopyParam(&layer[i], &input_layer[i]);
        alpha[i] = input_layer[i].alpha;
        memcpy(&transform_cfg[i], &input_layer[i].transform_cfg, sizeof(transform_cfg[0]));
    }
    EPIC_CopyParam(&layer[input_layer_num], output_layer);

    EPIC_MakeAllLayerCoordValidEx(layer, input_layer_num + 1);

    ret = EPIC_ConfigBlendEx(epic, layer, alpha, transform_cfg, input_layer_num + 1);
    if (HAL_OK != ret)
    {
        ret = HAL_OK;
        goto __EXIT;
    }


    epic->Instance->COMMAND |= EPIC_COMMAND_START;
    EPIC_WaitDone(epic);
#ifdef HAL_EZIP_MODULE_ENABLED
    while (HAL_OK != HAL_EZIP_CheckReady(epic->hezip))
    {
        // wait for ezip idle
    }
#endif

__EXIT:

    EPIC_DisableLayer(epic->Instance, EPIC_LAYER_IDX_0);
    EPIC_DisableLayer(epic->Instance, EPIC_LAYER_IDX_1);
    EPIC_DisableVideoLayer(epic->Instance);
    EPIC_DisableMaskLayer(epic->Instance);
    epic->Instance->CANVAS_BG = 0;

    EPIC_DISABLE(epic);

    epic->State = HAL_EPIC_STATE_READY;

    return HAL_OK;
}



HAL_StatusTypeDef HAL_EPIC_BlendStartEx_IT(EPIC_HandleTypeDef *epic,
        EPIC_LayerConfigTypeDef *input_layer,
        uint8_t input_layer_num,
        EPIC_LayerConfigTypeDef *output_layer)
{

    HAL_StatusTypeDef ret;
    EPIC_BlendingDataType layer[MAX_EPIC_LAYER + 1];  /*+1 for output_layer*/
    uint8_t alpha[MAX_EPIC_LAYER];
    EPIC_TransformCfgTypeDef transform_cfg[MAX_EPIC_LAYER];
    uint32_t i;

    HAL_ASSERT(NULL == epic->IntXferCpltCallback);


    if (HAL_EPIC_STATE_READY != epic->State)
    {
        RETURN_ERROR(epic, HAL_BUSY);
    }

    if (input_layer_num > MAX_EPIC_LAYER) //((input_layer_num > MAX_EPIC_LAYER) || (0 == input_layer_num))
    {
        RETURN_ERROR(epic, HAL_ERROR);
    }

    for (i = 0; i < input_layer_num; i++)
    {
        if (input_layer[i].width > input_layer[i].total_width)
        {
            RETURN_ERROR(epic, HAL_ERROR);
        }
    }

    if (!EPIC_SUPPROT_OUT_FORMAT(output_layer->color_mode)) RETURN_ERROR(epic, HAL_ERROR);

    EPIC_ENABLE(epic);

#ifdef EPIC_DEBUG
    EPIC_RecordBlendingExOp(epic, input_layer, input_layer_num, output_layer);
#endif

    epic->State = HAL_EPIC_STATE_BUSY;

    for (i = 0; i < input_layer_num; i++)
    {
        EPIC_CopyParam(&layer[i], &input_layer[i]);
        alpha[i] = input_layer[i].alpha;
        memcpy(&transform_cfg[i], &input_layer[i].transform_cfg, sizeof(transform_cfg[0]));
    }
    EPIC_CopyParam(&layer[input_layer_num], output_layer);

    EPIC_MakeAllLayerCoordValidEx(layer, input_layer_num + 1);

    ret = EPIC_ConfigBlendEx(epic, layer, alpha, transform_cfg, input_layer_num + 1);
    if (HAL_OK != ret)
    {
        EPIC_BlendCpltCallback(epic);
        return HAL_OK;
    }

    epic->IntXferCpltCallback = EPIC_BlendCpltCallback;

    /* clear pending interrupt */
    epic->Instance->EOF_IRQ = epic->Instance->EOF_IRQ;
    /* unmask */
    epic->Instance->SETTING |= EPIC_SETTING_EOF_IRQ_MASK;
    /* trigger epic */
    epic->Instance->COMMAND |= EPIC_COMMAND_START;

    return HAL_OK;

}




/**
 * @brief  Rotate and scale fg based on the image center, blend with bg and output to dst in polling mode
 *
 * @param  epic EPIC instance
 * @param  rot_cfg rotation configuration
 * @param  scale scale factor*EPIC_INPUT_SCALE_NONE, EPIC_INPUT_SCALE_NONE means no scaling,
           < EPIC_INPUT_SCALE_NONE means scaling up, > EPIC_INPUT_SCALE_NONE means scaling down
 * @param  fg configuration of foreground layer that's to be rotated
 * @param  bg background layer configuration
 * @param  dst output layer configuration
 *
 * @retval HAL status
 *
 * @note fields in fg, bg and dst would be modified
 */
HAL_StatusTypeDef HAL_EPIC_Rotate(EPIC_HandleTypeDef *epic, EPIC_TransformCfgTypeDef *rot_cfg,
                                  EPIC_BlendingDataType *fg, EPIC_BlendingDataType *bg,
                                  EPIC_BlendingDataType *dst, uint8_t alpha)
{
    HAL_StatusTypeDef ret = HAL_OK;
    EPIC_BlendingDataType bg_bak;
    EPIC_BlendingDataType fg_bak;
    EPIC_BlendingDataType dst_bak;
    EPIC_TransformCfgTypeDef rot_cfg_bak;

    if (!EPIC_IS_VALID_SCALE(rot_cfg->scale_x) || !EPIC_IS_VALID_SCALE(rot_cfg->scale_y))
    {
        RETURN_ERROR(epic, HAL_ERROR);
    }

#ifndef SF32LB55X
    if ((IS_ALPHA_MASK_LAYER(fg) && IS_NEED_TRANSFROM(rot_cfg)) //Mask layer can't transform
            || IS_ALPHA_MASK_LAYER(bg)//Mask layer have no target
       )
        RETURN_ERROR(epic, HAL_ERROR);
#endif /* SF32LB55X */

    if (!EPIC_SUPPROT_OUT_FORMAT(dst->color_mode)) RETURN_ERROR(epic, HAL_ERROR);

    if (HAL_EPIC_STATE_READY != epic->State)
    {
        RETURN_ERROR(epic, HAL_BUSY);
    }


    EPIC_ENABLE(epic);

#ifdef EPIC_DEBUG
    EPIC_RecordRotationOp(epic, rot_cfg, fg, bg, dst, alpha);
#endif

    epic->State = HAL_EPIC_STATE_BUSY;
    memcpy(&bg_bak,  bg,  sizeof(bg_bak));
    memcpy(&fg_bak,  fg,  sizeof(fg_bak));
    memcpy(&dst_bak, dst, sizeof(dst_bak));
    memcpy(&rot_cfg_bak, rot_cfg, sizeof(EPIC_TransformCfgTypeDef));

    EPIC_MakeAllLayerCoordValid(&fg_bak, &bg_bak, &dst_bak);

    if ((0 == dst_bak.width) || (0 == dst_bak.height))
    {
        ;//Nothing is need to do
    }
    else
    {
        ret = EPIC_ConfigRotation(epic, &rot_cfg_bak, &fg_bak, &bg_bak, &dst_bak, alpha);
        if (HAL_OK == ret)
        {
            epic->Instance->COMMAND |= EPIC_COMMAND_START;
            EPIC_WaitDone(epic);

#ifdef HAL_EZIP_MODULE_ENABLED
            while (HAL_OK != HAL_EZIP_CheckReady(epic->hezip))
            {
                // wait for ezip idle
            }
#endif
        }
        else if (HAL_EPIC_NOTHING_TO_DO == ret)
        {
            ret = HAL_OK;
        }
        else
        {
            /* Do nothing */
        }
    }

    EPIC_DisableLayer(epic->Instance, EPIC_LAYER_IDX_0);
    EPIC_DisableVideoLayer(epic->Instance);
    EPIC_DisableMaskLayer(epic->Instance);
    epic->Instance->CANVAS_BG = 0;

    EPIC_DISABLE(epic);

    //rt_mutex_release(&epic_mutex);

    epic->State = HAL_EPIC_STATE_READY;


    return ret;
}


/**
* @brief  Rotate and scale fg based on the image center, blend with bg and output to dst in interrupt mode
*
* @param  epic EPIC instance
* @param  rot_cfg rotation configuration
* @param  scale scale factor*EPIC_INPUT_SCALE_NONE, EPIC_INPUT_SCALE_NONE means no scaling,
   < EPIC_INPUT_SCALE_NONE means scaling up, > EPIC_INPUT_SCALE_NONE means scaling down
* @param  fg configuration of foreground layer that's to be rotated
* @param  bg background layer configuration
* @param  dst output layer configuration
* @param  alpha alpha value
*
* @retval HAL status
*
* @note fields in fg, bg and dst would be modified
*
*/
HAL_StatusTypeDef HAL_EPIC_Rotate_IT(EPIC_HandleTypeDef *epic, EPIC_TransformCfgTypeDef *rot_cfg,
                                     EPIC_BlendingDataType *fg, EPIC_BlendingDataType *bg,
                                     EPIC_BlendingDataType *dst, uint8_t alpha)
{
    EPIC_BlendingDataType bg_bak;
    EPIC_BlendingDataType fg_bak;
    EPIC_BlendingDataType dst_bak;
    EPIC_TransformCfgTypeDef rot_cfg_bak;
    HAL_StatusTypeDef ret;

    HAL_ASSERT(NULL == epic->IntXferCpltCallback);
    if (!EPIC_IS_VALID_SCALE(rot_cfg->scale_x) || !EPIC_IS_VALID_SCALE(rot_cfg->scale_y))
    {
        RETURN_ERROR(epic, HAL_ERROR);
    }

#ifndef SF32LB55X
    if ((IS_ALPHA_MASK_LAYER(fg) && IS_NEED_TRANSFROM(rot_cfg)) //Mask layer can't transform
            || IS_ALPHA_MASK_LAYER(bg)//Mask layer have no target
       )
        RETURN_ERROR(epic, HAL_ERROR);
#endif /* SF32LB55X */

    if (!EPIC_SUPPROT_OUT_FORMAT(dst->color_mode)) RETURN_ERROR(epic, HAL_ERROR);

    if (HAL_EPIC_STATE_READY != epic->State)
    {
        RETURN_ERROR(epic, HAL_BUSY);
    }

    EPIC_ENABLE(epic);

#ifdef EPIC_DEBUG
    EPIC_RecordRotationOp(epic, rot_cfg, fg, bg, dst, alpha);
#endif

    epic->State = HAL_EPIC_STATE_BUSY;

    memcpy(&bg_bak,  bg,  sizeof(bg_bak));
    memcpy(&fg_bak,  fg,  sizeof(fg_bak));
    memcpy(&dst_bak, dst, sizeof(dst_bak));
    memcpy(&rot_cfg_bak, rot_cfg, sizeof(EPIC_TransformCfgTypeDef));


    EPIC_MakeAllLayerCoordValid(&fg_bak, &bg_bak, &dst_bak);

    if ((0 == dst_bak.width) || (0 == dst_bak.height))
    {
        ;//Nothing is need to do
        EPIC_RotationCpltCallback(epic);
    }
    else
    {
        ret = EPIC_ConfigRotation(epic, &rot_cfg_bak, &fg_bak, &bg_bak, &dst_bak, alpha);
        if (HAL_OK == ret)
        {
            epic->IntXferCpltCallback = EPIC_RotationCpltCallback;

            /* clear pending interrupt */
            epic->Instance->EOF_IRQ = epic->Instance->EOF_IRQ;
            /* unmask */
            epic->Instance->SETTING |= EPIC_SETTING_EOF_IRQ_MASK;

            /* trigger epic */
            epic->Instance->COMMAND |= EPIC_COMMAND_START;
        }
        else if (HAL_EPIC_NOTHING_TO_DO == ret)
        {
            EPIC_RotationCpltCallback(epic);
            ret = HAL_OK;
        }
        else
        {
            return ret;
        }
    }

    //rt_mutex_release(&epic_mutex);

    return HAL_OK;
}


/**
* @brief  Init an epic fill cfg struct to default vaules
*
* @param[layer]  EPIC layer
* @retval None
*/
void HAL_EPIC_FillDataInit(EPIC_FillingCfgTypeDef *param)
{
    HAL_ASSERT(NULL != param);
    memset(param, 0, sizeof(EPIC_FillingCfgTypeDef));
}

/**
 * @brief  Fill the region with given color in interrupt mode
 *
 *  if alpha is used, filling color would be blended with original color in the buffer
 *  output_color = alpha * filling_color + (255 - alpha) * original_color
 *
 * @param  epic EPIC instance
 * @param  param filling region configuration
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EPIC_FillStart_IT(EPIC_HandleTypeDef *epic, EPIC_FillingCfgTypeDef *param)
{
    HAL_StatusTypeDef ret;

    HAL_ASSERT(param);
    HAL_ASSERT(NULL == epic->IntXferCpltCallback);

    if (!EPIC_SUPPROT_OUT_FORMAT(param->color_mode)) RETURN_ERROR(epic, HAL_ERROR);
    if (HAL_EPIC_STATE_READY != epic->State)
    {
        RETURN_ERROR(epic, HAL_BUSY);
    }

    EPIC_ENABLE(epic);

#ifdef EPIC_DEBUG
    EPIC_RecordFillingOp(epic, param);
#endif

    epic->State = HAL_EPIC_STATE_BUSY;

    ret = EPIC_ConfigFilling(epic, param);

    if (HAL_OK != ret)
    {
        EPIC_FillCpltCallback(epic);
        goto __EXIT;

    }

    epic->IntXferCpltCallback = EPIC_FillCpltCallback;

    /* clear pending interrupt */
    epic->Instance->EOF_IRQ = epic->Instance->EOF_IRQ;
    /* unmask */
    epic->Instance->SETTING |= EPIC_SETTING_EOF_IRQ_MASK;

    /* trigger epic */
    epic->Instance->COMMAND |= EPIC_COMMAND_START;

__EXIT:

    return HAL_OK;
}

HAL_StatusTypeDef HAL_EPIC_FillStart(EPIC_HandleTypeDef *epic, EPIC_FillingCfgTypeDef *param)
{
    HAL_StatusTypeDef ret;

    HAL_ASSERT(param);
    HAL_ASSERT(NULL == epic->IntXferCpltCallback);
    if (!EPIC_SUPPROT_OUT_FORMAT(param->color_mode)) RETURN_ERROR(epic, HAL_ERROR);

    if (HAL_EPIC_STATE_READY != epic->State)
    {
        RETURN_ERROR(epic, HAL_BUSY);
    }

    EPIC_ENABLE(epic);

#ifdef EPIC_DEBUG
    EPIC_RecordFillingOp(epic, param);
#endif

    epic->State = HAL_EPIC_STATE_BUSY;

    ret = EPIC_ConfigFilling(epic, param);
    if (HAL_OK != ret)
    {
        goto __EXIT;
    }

    /* trigger epic */
    epic->Instance->COMMAND |= EPIC_COMMAND_START;

    /* wait complete */
    EPIC_WaitDone(epic);

__EXIT:

    /* disable video layer */
    EPIC_DisableVideoLayer(epic->Instance);
    epic->Instance->CANVAS_BG = 0;

    EPIC_DISABLE(epic);

    epic->State = HAL_EPIC_STATE_READY;

    return HAL_OK;
}



HAL_StatusTypeDef HAL_EPIC_Copy_IT(EPIC_HandleTypeDef *epic, EPIC_BlendingDataType *src, EPIC_BlendingDataType *dst)
{
    EPIC_BlendingDataType src_bak;
    EPIC_BlendingDataType dst_bak;
    HAL_StatusTypeDef ret;
    EPIC_LAYER_IDX src_layer_idx;
    EPIC_TransformResultDef trans_result;

#if 0
#ifndef SF32LB52X
    EPIC_TransformResultDef trans_result;
#endif /* SF32LB52X */
#endif /* if 0 */

    HAL_ASSERT(NULL == epic->IntXferCpltCallback);
    if (!EPIC_SUPPROT_OUT_FORMAT(dst->color_mode)) RETURN_ERROR(epic, HAL_ERROR);

    if (HAL_EPIC_STATE_READY != epic->State)
    {
        RETURN_ERROR(epic, HAL_BUSY);
    }

    EPIC_ENABLE(epic);

#ifdef EPIC_DEBUG
    EPIC_RecordCopyOp(epic, src, dst);
#endif

    epic->State = HAL_EPIC_STATE_BUSY;

    memcpy(&src_bak, src, sizeof(src_bak));
    memcpy(&dst_bak, dst, sizeof(dst_bak));

#ifndef SF32LB52X
    src_layer_idx = EPIC_LAYER_IDX_1;

#else
    /* L0 use same port as output,
       VL use different port from output but it need two cycles per pixcel,
       L0 is a little faster than VL on FPGA, need check the performance on chip when clock is raised
     */
    src_layer_idx = EPIC_LAYER_IDX_0;

#if 0
    EPIC_TransResultInit(&trans_result);
    trans_result.width  = src_bak.width;
    trans_result.height = src_bak.height;

    ret = EPIC_ConfigVideoLayer(epic, EPIC_LAYER_IDX_VL, &trans_result, &src_bak, 255);
#endif /* if 0 */
#endif /* SF32LB52X */

    EPIC_MakeAllLayerCoordValid(&src_bak, &dst_bak, NULL);

    EPIC_TransResultInit(&trans_result, &src_bak);
    EPIC_ClipLayerSrcByOutput(&src_bak, NULL, &dst_bak, &trans_result);

    EPIC_MakeAllLayerCoordValid(&src_bak, &dst_bak, NULL);

    ret = EPIC_ConfigLayer(epic, src_layer_idx, &src_bak, true, 255);

    if (HAL_OK != ret)
    {
        goto __EXIT;
    }

    ret = EPIC_ConfigOutputLayer(epic->Instance, &src_bak, &dst_bak, &dst_bak);
    if (HAL_OK != ret)
    {
        epic->ErrorCode = __LINE__;
        goto __EXIT;
    }

#ifdef HAL_EZIP_MODULE_ENABLED
    if (IS_EZIP_COLOR_MODE(src_bak.color_mode) && EPIC_IsLayerActive(epic->Instance, src_layer_idx))
    {
        ret = EPIC_ConfigEzipDec(epic, &src_bak, &dst_bak, &trans_result);
    }
#endif /* HAL_EZIP_MODULE_ENABLED */

#ifndef SF32LB55X
    epic->Instance->CANVAS_BG = 0;
    epic->Instance->CANVAS_BG |= EPIC_CANVAS_BG_ALL_BLENDING_BYPASS;
#else
    epic->Instance->CANVAS_BG = 0;
#endif /* SF32LB55X */

    epic->IntXferCpltCallback = EPIC_CopyCpltCallback;

    /* clear pending interrupt */
    epic->Instance->EOF_IRQ = epic->Instance->EOF_IRQ;
    /* unmask */
    epic->Instance->SETTING |= EPIC_SETTING_EOF_IRQ_MASK;

    //HAL_ASSERT(0);

    /* trigger epic */
    epic->Instance->COMMAND |= EPIC_COMMAND_START;

    //rt_mutex_release(&epic_mutex);

__EXIT:

    if (ret != HAL_OK)
    {
        EPIC_CopyCpltCallback(epic);
    }

    return HAL_OK;
}


/**
* @brief  Init an epic fill grad cfg struct to default vaules
*
* @param[layer]  EPIC layer
* @retval None
*/
void HAL_EPIC_FillGradDataInit(EPIC_GradCfgTypeDef *param)
{
    HAL_ASSERT(NULL != param);
    memset(param, 0, sizeof(EPIC_GradCfgTypeDef));
}

HAL_StatusTypeDef HAL_EPIC_FillGrad_IT(EPIC_HandleTypeDef *epic, EPIC_GradCfgTypeDef *param)
{
    HAL_StatusTypeDef ret;

    HAL_ASSERT(param);
    HAL_ASSERT(NULL == epic->IntXferCpltCallback);
    if (!EPIC_SUPPROT_OUT_FORMAT(param->color_mode)) RETURN_ERROR(epic, HAL_ERROR);

    if (HAL_EPIC_STATE_READY != epic->State)
    {
        RETURN_ERROR(epic, HAL_BUSY);
    }

    EPIC_ENABLE(epic);

#ifdef EPIC_DEBUG
    EPIC_RecordGradOp(epic, param);
#endif

    epic->State = HAL_EPIC_STATE_BUSY;

    ret = EPIC_ConfigGrad(epic, param);

    if (HAL_OK != ret)
    {
        EPIC_GradCpltCallback(epic);
        goto __EXIT;

    }

    epic->IntXferCpltCallback = EPIC_GradCpltCallback;

    /* clear pending interrupt */
    epic->Instance->EOF_IRQ = epic->Instance->EOF_IRQ;
    /* unmask */
    epic->Instance->SETTING |= EPIC_SETTING_EOF_IRQ_MASK;

    /* trigger epic */
    epic->Instance->COMMAND |= EPIC_COMMAND_START;

__EXIT:

    return HAL_OK;
}


HAL_StatusTypeDef HAL_EPIC_FillGrad(EPIC_HandleTypeDef *epic, EPIC_GradCfgTypeDef *param)
{
    HAL_StatusTypeDef ret;

    HAL_ASSERT(param);
    HAL_ASSERT(NULL == epic->IntXferCpltCallback);
    if (!EPIC_SUPPROT_OUT_FORMAT(param->color_mode)) RETURN_ERROR(epic, HAL_ERROR);

    if (HAL_EPIC_STATE_READY != epic->State)
    {
        RETURN_ERROR(epic, HAL_BUSY);
    }

    EPIC_ENABLE(epic);

#ifdef EPIC_DEBUG
    EPIC_RecordGradOp(epic, param);
#endif

    epic->State = HAL_EPIC_STATE_BUSY;

    ret = EPIC_ConfigGrad(epic, param);

    if (HAL_OK != ret)
    {
        EPIC_GradCpltCallback(epic);
        goto __EXIT;

    }

    /* trigger epic */
    epic->Instance->COMMAND |= EPIC_COMMAND_START;

    /* wait complete */
    EPIC_WaitDone(epic);

__EXIT:

    /* disable video layer */
    EPIC_DisableVideoLayer(epic->Instance);

    epic->State = HAL_EPIC_STATE_READY;

    return HAL_OK;
}

static HAL_StatusTypeDef EPIC_ContConfigXLayer(EPIC_HandleTypeDef *hepic,
        uint32_t idx, const EPIC_LayerConfigTypeDef *config)
{
    HAL_StatusTypeDef ret_v;

    bool is_videolayer;

    EPIC_LAYER_IDX layer_idx = hepic->api_cfg.cont_cfg.epic_layer[idx];
#ifdef SF32LB55X
    is_videolayer = (EPIC_LAYER_IDX_VL == layer_idx);
#else
    is_videolayer = (layer_idx >= EPIC_LAYER_IDX_2);
#endif /* SF32LB55X */


    if (is_videolayer)
        ret_v = EPIC_ContConfigVideoLayer(hepic, idx, config);
    else
        ret_v = EPIC_ContConfigLayer(hepic, idx, config);


    return ret_v;
}


HAL_StatusTypeDef HAL_EPIC_ContBlendStart(EPIC_HandleTypeDef *hepic,
        EPIC_LayerConfigTypeDef *input_layer,
        EPIC_LayerConfigTypeDef *mask_layer,
        EPIC_LayerConfigTypeDef *output_layer)
{

    HAL_StatusTypeDef ret_v;

    HAL_ASSERT(NULL == hepic->IntXferCpltCallback);

    if (HAL_EPIC_STATE_READY != hepic->State)
    {
        RETURN_ERROR(hepic, HAL_BUSY);
    }

    if (input_layer->width > input_layer->total_width)
    {
        RETURN_ERROR(hepic, HAL_ERROR);
    }
#ifdef HAL_EZIP_MODULE_ENABLED
    if (IS_EZIP_COLOR_MODE(input_layer->color_mode))
    {
        RETURN_ERROR(hepic, HAL_ERROR);//Not support EZIP now
    }
#endif

    if (IS_NEED_TRANSFROM(&input_layer->transform_cfg))
    {
        RETURN_ERROR(hepic, HAL_ERROR); //Not support transform too.
    }

    if (!EPIC_SUPPROT_OUT_FORMAT(output_layer->color_mode)) RETURN_ERROR(hepic, HAL_ERROR);

    EPIC_ENABLE(hepic);

#ifdef EPIC_DEBUG
    EPIC_RecordContBlendingOp(hepic, input_layer, mask_layer, output_layer);
#endif

    hepic->api_type = EPIC_API_CONTINUE;
    hepic->State = HAL_EPIC_STATE_BUSY;
    hepic->api_cfg.cont_cfg.epic_layer[CONT_FG_LAYER] = EPIC_LAYER_IDX_VL;
    hepic->api_cfg.cont_cfg.epic_layer[CONT_BG_LAYER] = EPIC_LAYER_IDX_0;
    hepic->api_cfg.cont_cfg.color_format[CONT_FG_LAYER] = EPIC_GetLayerColorFormat(input_layer->color_mode);
    hepic->api_cfg.cont_cfg.color_format[CONT_BG_LAYER] = EPIC_GetLayerColorFormat(output_layer->color_mode);
    hepic->api_cfg.cont_cfg.color_depth[CONT_FG_LAYER] = EPIC_GetColorDepth(input_layer->color_mode);
    hepic->api_cfg.cont_cfg.color_depth[CONT_BG_LAYER] = EPIC_GetColorDepth(output_layer->color_mode);

    //Move one layer's TL to (0,0)
    int16_t offset_x, offset_y;
    EPIC_GetLayersCoordMin((EPIC_BlendingDataType *)input_layer,
                           (EPIC_BlendingDataType *)mask_layer,
                           (EPIC_BlendingDataType *)output_layer,
                           &offset_x, &offset_y);
    EPIC_MoveLayers((EPIC_BlendingDataType *)input_layer,
                    (EPIC_BlendingDataType *)mask_layer,
                    (EPIC_BlendingDataType *)output_layer,
                    -offset_x, -offset_y);

    //FG
    ret_v = EPIC_ContConfigVideoLayer(hepic, CONT_FG_LAYER, input_layer);
    if (HAL_OK != ret_v) goto __EXIT;
    EPIC_ContResetVideoLayer(hepic, CONT_FG_LAYER); //Reset transform configurations


    //BG
    ret_v = EPIC_ContConfigLayer(hepic, CONT_BG_LAYER, output_layer);
    if (HAL_OK != ret_v) goto __EXIT;


    //MASK
    if (mask_layer)
    {
#ifdef EPIC_SUPPORT_MASK
        hepic->api_cfg.cont_cfg.color_format[CONT_MASK_LAYER] = EPIC_GetMaskLayerColorFormat(mask_layer->color_mode);
#endif /* EPIC_SUPPORT_MASK */
        hepic->api_cfg.cont_cfg.color_depth[CONT_MASK_LAYER] = EPIC_GetColorDepth(mask_layer->color_mode);
        ret_v = EPIC_ContConfigMaskLayer(hepic, CONT_FG_LAYER, (EPIC_BlendingDataType *)mask_layer);
        if (HAL_OK != ret_v) goto __EXIT;
    }


//    ret_v = EPIC_ContConfigOutputLayer(hepic, output_layer);
    ret_v = EPIC_ConfigOutputLayerEx(hepic, (EPIC_BlendingDataType *)input_layer, 1, (EPIC_BlendingDataType *)output_layer);
    if (HAL_OK != ret_v) goto __EXIT;



    hepic->Instance->COMMAND |= EPIC_COMMAND_START;


__EXIT:
    //Restore
    EPIC_MoveLayers((EPIC_BlendingDataType *)input_layer,
                    (EPIC_BlendingDataType *)mask_layer,
                    (EPIC_BlendingDataType *)output_layer,
                    offset_x, offset_y);

    return ret_v;
}




HAL_StatusTypeDef HAL_EPIC_ContBlendRepeat(EPIC_HandleTypeDef *hepic,
        EPIC_LayerConfigTypeDef *input_layer,
        EPIC_LayerConfigTypeDef *mask_layer,
        EPIC_LayerConfigTypeDef *output_layer)
{
    HAL_StatusTypeDef ret_v = HAL_OK;


    if ((HAL_EPIC_STATE_BUSY != hepic->State) || (EPIC_API_CONTINUE != hepic->api_type) || (!input_layer))
    {
        RETURN_ERROR(hepic, HAL_ERROR);
    }

#ifdef EPIC_CONT_BLEND_DEBUG
#ifdef EPIC_DEBUG
    EPIC_RecordContBlendingRepeatOp(hepic, input_layer, mask_layer, output_layer);
#endif
#endif /* EPIC_CONT_BLEND_DEBUG */

    if (input_layer->width && input_layer->height && input_layer->total_width)
    {
        //Move one layer's TL to (0,0)
        int16_t offset_x, offset_y;
        EPIC_GetLayersCoordMin((EPIC_BlendingDataType *)input_layer,
                               (EPIC_BlendingDataType *)mask_layer,
                               (EPIC_BlendingDataType *)output_layer,
                               &offset_x, &offset_y);
        EPIC_MoveLayers((EPIC_BlendingDataType *)input_layer,
                        (EPIC_BlendingDataType *)mask_layer,
                        (EPIC_BlendingDataType *)output_layer,
                        -offset_x, -offset_y);

        EPIC_WaitDone(hepic);

        //FG
        ret_v = EPIC_ContConfigXLayer(hepic, CONT_FG_LAYER, input_layer);
        if (HAL_OK != ret_v) goto __EXIT;



        //BG
        ret_v = EPIC_ContConfigXLayer(hepic, CONT_BG_LAYER, output_layer);
        if (HAL_OK != ret_v) goto __EXIT;


        //MASK
        if (mask_layer)
        {
            ret_v = EPIC_ContConfigMaskLayer(hepic, CONT_FG_LAYER, (EPIC_BlendingDataType *)mask_layer);
            if (HAL_OK != ret_v) goto __EXIT;
        }


        ret_v = EPIC_ContConfigOutputLayer(hepic, output_layer);
        if (HAL_OK != ret_v) goto __EXIT;

        hepic->Instance->COMMAND |= EPIC_COMMAND_START;

__EXIT:
        //Restore
        EPIC_MoveLayers((EPIC_BlendingDataType *)input_layer,
                        (EPIC_BlendingDataType *)mask_layer,
                        (EPIC_BlendingDataType *)output_layer,
                        offset_x, offset_y);

    }


    return ret_v;
}

HAL_StatusTypeDef HAL_EPIC_ContBlendStop(EPIC_HandleTypeDef *hepic)
{
    if (EPIC_API_CONTINUE == hepic->api_type)
    {
        //while(hepic->Instance->STATUS != 0);
        EPIC_WaitDone(hepic);


        EPIC_DisableLayer(hepic->Instance, EPIC_LAYER_IDX_0);
        EPIC_DisableLayer(hepic->Instance, EPIC_LAYER_IDX_1);
        EPIC_DisableVideoLayer(hepic->Instance);
        EPIC_DisableMaskLayer(hepic->Instance);
        hepic->Instance->CANVAS_BG = 0;

        EPIC_DISABLE(hepic);

        hepic->api_type = EPIC_API_NORMAL;
        hepic->State = HAL_EPIC_STATE_READY;
        return HAL_OK;
    }
    else
    {
        return HAL_OK;
    }

}

bool HAL_EPIC_IsHWBusy(EPIC_HandleTypeDef *hepic)
{
    return (hepic->Instance->STATUS != 0x0);
}

HAL_StatusTypeDef HAL_EPIC_TransStart(EPIC_HandleTypeDef *hepic,
                                      EPIC_LayerConfigTypeDef *input_layer,
                                      uint8_t input_layer_num,
                                      EPIC_LayerConfigTypeDef *output_layer,
                                      EPIC_TransPath hor_path,
                                      EPIC_TransPath ver_path,
                                      void *user_data)
{
    HAL_StatusTypeDef ret;
    EPIC_BlendingDataType layer[MAX_EPIC_LAYER + 1];  /*+1 for output_layer*/
    uint8_t alpha[MAX_EPIC_LAYER];
    EPIC_TransformCfgTypeDef transform_cfg[MAX_EPIC_LAYER];
    uint32_t i, pixel_bytes, row_bytes;
    int16_t start_x, start_y, end_x, end_y, loop_start_x, loop_end_x, loop_start_y, loop_end_y;
    uint8_t *data_head;

    HAL_ASSERT(NULL == hepic->IntXferCpltCallback);


    if (HAL_EPIC_STATE_READY != hepic->State)
    {
        RETURN_ERROR(hepic, HAL_BUSY);
    }

    if ((input_layer_num > MAX_EPIC_LAYER) || (0 == input_layer_num) || (NULL == hor_path) || (NULL == ver_path))
    {
        RETURN_ERROR(hepic, HAL_ERROR);
    }

    for (i = 0; i < input_layer_num; i++)
    {
        if (input_layer[i].width > input_layer[i].total_width)
        {
            RETURN_ERROR(hepic, HAL_ERROR);
        }
    }
    if (!EPIC_SUPPROT_OUT_FORMAT(output_layer->color_mode)) RETURN_ERROR(hepic, HAL_ERROR);

    EPIC_ENABLE(hepic);
    EPIC_InitRamInstance(hepic);

#ifdef EPIC_DEBUG
    EPIC_RecordTransformOp(hepic, input_layer, input_layer_num,
                           output_layer, hor_path, ver_path, user_data);
#endif

    hepic->State = HAL_EPIC_STATE_BUSY;

    hepic->api_type = EPIC_API_TRANSFORM;
    hepic->api_cfg.trans_cfg.hor_path = hor_path;
    hepic->api_cfg.trans_cfg.ver_path = ver_path;
    hepic->api_cfg.trans_cfg.user_data = user_data;

    start_x = output_layer->x_offset;
    start_y = output_layer->y_offset;
    loop_start_x = start_x;
    loop_start_y = start_y;
    loop_end_x = output_layer->x_offset + output_layer->width - 1;
    loop_end_y = output_layer->y_offset + output_layer->height - 1;
    pixel_bytes = EPIC_GetColorDepth(output_layer->color_mode) >> 3;
    row_bytes = pixel_bytes * output_layer->total_width;
    data_head = output_layer->data;
    while (start_y <= loop_end_y)
    {
        end_y = ver_path(start_y, &input_layer[1], user_data);
        HAL_ASSERT(end_y >= start_y);
        end_y = EPIC_MATH_MIN(end_y, loop_end_y);
        while (start_x <= loop_end_x)
        {
            end_x = hor_path(start_x, &input_layer[1], user_data);
            HAL_ASSERT(end_x >= start_x);
            end_x = EPIC_MATH_MIN(end_x, loop_end_x);

            //Update ouput area {[start_x,end_x],[start_y,end_y]}
            output_layer->width     = end_x - start_x + 1;
            output_layer->height    = end_y - start_y + 1;

            //Perform transformation
            {
                for (i = 0; i < input_layer_num; i++)
                {
                    EPIC_CopyParam(&layer[i], &input_layer[i]);
                    alpha[i] = input_layer[i].alpha;
                    memcpy(&transform_cfg[i], &input_layer[i].transform_cfg, sizeof(transform_cfg[0]));
                }
                EPIC_CopyParam(&layer[input_layer_num], output_layer);

                EPIC_MakeAllLayerCoordValidEx(layer, input_layer_num + 1);

                //Wait previous action done.
                {
                    EPIC_WaitValidInstance(hepic);

#ifdef HAL_EZIP_MODULE_ENABLED
                    while (HAL_OK != HAL_EZIP_CheckReady(hepic->hezip))
                    {
                        // wait for ezip idle
                    }
#endif
                }

                ret = EPIC_ConfigBlendEx(hepic, layer, alpha, transform_cfg, input_layer_num + 1);

                if (HAL_EPIC_NOTHING_TO_DO == ret)
                {
                    EPIC_DisposeRamInstance(hepic);
                    EPIC_PRINTF("HAL_EPIC_NOTHING_TO_DO= %d \n", start_y);
                }
                else if (HAL_OK == ret)
                {
                    EPIC_CommitInstance(hepic);
                    hepic->Instance->COMMAND |= EPIC_COMMAND_START;
                }
                else
                {
                    EPIC_DisposeRamInstance(hepic);
                    ret = HAL_OK;
                    goto __EXIT;
                }
            }

            //Move output layer to next colums
            output_layer->data      +=  pixel_bytes * output_layer->width;
            start_x = end_x + 1;
            output_layer->x_offset = start_x;
        }

        //Move output layer to next rows
        start_x = loop_start_x;
        output_layer->x_offset = start_x;

        start_y = end_y + 1;
        output_layer->y_offset = start_y;
        output_layer->data = data_head + (row_bytes * (start_y - loop_start_y));
    }


    EPIC_WaitDone(hepic);
#ifdef HAL_EZIP_MODULE_ENABLED
    while (HAL_OK != HAL_EZIP_CheckReady(hepic->hezip))
    {
        // wait for ezip idle
    }
#endif

__EXIT:

    EPIC_DisableLayer(hepic->Instance, EPIC_LAYER_IDX_0);
    EPIC_DisableLayer(hepic->Instance, EPIC_LAYER_IDX_1);
    EPIC_DisableVideoLayer(hepic->Instance);
    EPIC_DisableMaskLayer(hepic->Instance);
    hepic->Instance->CANVAS_BG = 0;
    hepic->api_type = EPIC_API_NORMAL;

    EPIC_DISABLE(hepic);

    hepic->State = HAL_EPIC_STATE_READY;

    return HAL_OK;
}


void HAL_EPIC_IRQHandler(EPIC_HandleTypeDef *epic)
{
    void (*cb)(EPIC_HandleTypeDef * epic);

    epic->Instance->EOF_IRQ = epic->Instance->EOF_IRQ;
    epic->Instance->SETTING &= ~EPIC_SETTING_EOF_IRQ_MASK;

    cb = epic->IntXferCpltCallback;
    if (cb)
    {
        cb(epic);
    }
}


void HAL_EPIC_LayerSetDataOffset(EPIC_BlendingDataType *layer, int16_t x, int16_t y)
{
    int32_t pixel_depth = EPIC_GetColorDepth(layer->color_mode);

    if (0 == (pixel_depth & 0x7))
    {
        int32_t offset_pixels = ((int32_t)(y - layer->y_offset)) * layer->total_width;

        offset_pixels += x - layer->x_offset;

        layer->data += (pixel_depth >> 3) * offset_pixels;
    }
    else
    {
        int32_t offset_bytes, x_offset_bits;

        x_offset_bits = (x - layer->x_offset) * pixel_depth;

        if (x_offset_bits & 0x7)
        {
            return;
        }
        else
        {

            int32_t line_bytes = HAL_ALIGN(layer->total_width * pixel_depth, 8) >> 3;

            offset_bytes = x_offset_bits >> 3;

            offset_bytes += ((int32_t)(y - layer->y_offset)) * line_bytes;

            layer->data += offset_bytes;
        }

    }
}

HAL_StatusTypeDef HAL_EPIC_BlendFastStart_Init(EPIC_HandleTypeDef *hepic)
{
    hepic->api_type = EPIC_API_NORMAL;
    hepic->RamInstance_used = 1;
    hepic->Instance = hepic->RamInstance;
    hepic->State = HAL_EPIC_STATE_READY;
    hepic->IntXferCpltCallback = NULL;
    hepic->XferCpltCallback = NULL;

    EPIC_DisableLayer(hepic->Instance, EPIC_LAYER_IDX_0);
    EPIC_DisableLayer(hepic->Instance, EPIC_LAYER_IDX_1);
    EPIC_DisableLayer(hepic->Instance, EPIC_LAYER_IDX_2);
    EPIC_DisableVideoLayer(hepic->Instance);
    EPIC_DisableMaskLayer(hepic->Instance);
    hepic->Instance->CANVAS_BG = 0;


#ifdef HAL_EZIP_MODULE_ENABLED
    HAL_EZIP_DecodeFast_Init(hepic->hezip);
#endif /* HAL_EZIP_MODULE_ENABLED */

    return HAL_OK;
}

HAL_StatusTypeDef HAL_EPIC_BlendFastStart_IT(EPIC_HandleTypeDef *hepic, EPIC_HandleTypeDef *hepic_s)
{
    HAL_StatusTypeDef res = HAL_OK;

    HAL_ASSERT(NULL != hepic->HwInstance);
    HAL_ASSERT(NULL != hepic->RamInstance);


    if (HAL_EPIC_STATE_READY != hepic->State)
    {
        RETURN_ERROR(hepic, HAL_BUSY);
    }
    hepic->State = HAL_EPIC_STATE_BUSY;

    hepic->api_type = hepic_s->api_type;
    hepic->api_cfg = hepic_s->api_cfg;
    hepic->RamInstance_used = 1;
#ifdef EPIC_SUPPORT_L8
    hepic->RamLTabSize[0] = hepic_s->RamLTabSize[0];
    hepic->RamLTab[0]  = hepic_s->RamLTab[0];
#if EPIC_LOOKUP_TABLES > 1
    hepic->RamLTabSize[1] = hepic_s->RamLTabSize[1];
    hepic->RamLTab[1]  = hepic_s->RamLTab[1];
#endif /* EPIC_LOOKUP_TABLES > 1 */
#endif /* EPIC_SUPPORT_L8 */





    hepic->Instance = hepic->HwInstance;
    EPIC_ENABLE(hepic);

    EPIC_CommitInstance(hepic);
#ifdef HAL_EZIP_MODULE_ENABLED
    if ((int)HAL_EPIC_STATE_BUSY == (int)hepic_s->hezip->State)
    {
        hepic->hezip->CpltCallback = hepic_s->hezip->CpltCallback;
        hepic->hezip->RamInstance_used = 1;
        hepic->hezip->user_data = (void *)hepic;
        hepic_s->hezip->user_data = NULL; // clear user_data in hezip_s
        res = HAL_EZIP_DecodeFast_IT(hepic->hezip);
        HAL_ASSERT(res == HAL_OK);
    }
#endif /* HAL_EZIP_MODULE_ENABLED */

    hepic->user_data = hepic_s->user_data;
    hepic->IntXferCpltCallback = hepic_s->IntXferCpltCallback;
    hepic->XferCpltCallback = hepic_s->XferCpltCallback;
    /* clear pending interrupt */
    hepic->Instance->EOF_IRQ = hepic->Instance->EOF_IRQ;
    /* unmask */
    hepic->Instance->SETTING |= EPIC_SETTING_EOF_IRQ_MASK;
    /* trigger hepic */
    hepic->Instance->COMMAND |= EPIC_COMMAND_START;

    return HAL_OK;
}

static void EPIC_CommonCbk(EPIC_HandleTypeDef *epic)
{
    EPIC_CpltCallback usr_cbk;

    epic->Instance->CANVAS_BG = 0;

    EPIC_DISABLE(epic);

    usr_cbk = epic->XferCpltCallback;
    epic->XferCpltCallback = NULL;
    epic->IntXferCpltCallback = NULL;
    epic->State = HAL_EPIC_STATE_READY;

    //Allow calling HAL_EPIC API in usr_cbk
    if (usr_cbk)
    {
        usr_cbk(epic);
    }
}

static void EPIC_BlendCpltCallback(EPIC_HandleTypeDef *epic)
{
    EPIC_DisableLayer(epic->Instance, EPIC_LAYER_IDX_0);
    EPIC_DisableLayer(epic->Instance, EPIC_LAYER_IDX_1);
    EPIC_DisableLayer(epic->Instance, EPIC_LAYER_IDX_2);
    EPIC_DisableVideoLayer(epic->Instance);
    EPIC_DisableMaskLayer(epic->Instance);

#ifdef HAL_EZIP_MODULE_ENABLED
    if (HAL_OK != HAL_EZIP_CheckReady(epic->hezip))
    {
        return;
    }
#endif

    EPIC_CommonCbk(epic);
}

static void EPIC_FillCpltCallback(EPIC_HandleTypeDef *epic)
{
    /* disable video layer */
    EPIC_DisableVideoLayer(epic->Instance);
    EPIC_CommonCbk(epic);
}

static void EPIC_GradCpltCallback(EPIC_HandleTypeDef *epic)
{
    /* disable video layer */
    EPIC_DisableVideoLayer(epic->Instance);
    EPIC_DisableLayer(epic->Instance, EPIC_LAYER_IDX_0);
    EPIC_CommonCbk(epic);
}


static void EPIC_RotationCpltCallback(EPIC_HandleTypeDef *epic)
{
    EPIC_DisableLayer(epic->Instance, EPIC_LAYER_IDX_0);
    EPIC_DisableVideoLayer(epic->Instance);
    EPIC_DisableMaskLayer(epic->Instance);

#ifdef HAL_EZIP_MODULE_ENABLED
    if (HAL_OK != HAL_EZIP_CheckReady(epic->hezip))
    {
        return;
    }
#endif

    EPIC_CommonCbk(epic);
}

static void EPIC_CopyCpltCallback(EPIC_HandleTypeDef *epic)
{
#ifndef SF32LB52X
    /* disable layer 1 */
    EPIC_DisableLayer(epic->Instance, EPIC_LAYER_IDX_1);
#else
    /* disable layer 0 */
    EPIC_DisableLayer(epic->Instance, EPIC_LAYER_IDX_0);
#if 0
    /* disable video layer */
    EPIC_DisableVideoLayer(epic->Instance);
#endif /* if 0 */
#endif /* SF32LB52X */
#ifndef SF32LB55X
    epic->Instance->CANVAS_BG &= ~EPIC_CANVAS_BG_ALL_BLENDING_BYPASS;
#endif
    epic->Instance->CANVAS_BG = 0;

#ifdef HAL_EZIP_MODULE_ENABLED
    if (HAL_OK != HAL_EZIP_CheckReady(epic->hezip))
    {
        return;
    }
#endif /* HAL_EZIP_MODULE_ENABLED */

    EPIC_CommonCbk(epic);
}

#ifdef HAL_EZIP_MODULE_ENABLED
static void EPIC_EzipCpltCallback(EZIP_HandleTypeDef *ezip)
{
    EPIC_HandleTypeDef *hepic;

    hepic = (EPIC_HandleTypeDef *)ezip->user_data;
    HAL_ASSERT(NULL != hepic);
    ezip->user_data = NULL;
    if ((0 == (hepic->Instance->SETTING & EPIC_SETTING_EOF_IRQ_MASK)) /* epic has finished */
            && (hepic->IntXferCpltCallback))
    {
        hepic->IntXferCpltCallback(hepic);
    }
}
#endif /* HAL_EZIP_MODULE_ENABLED */

/**
 * @brief  Rotate fg, blend with bg and output to dst in polling mode
 *
 * @param  epic EPIC instance
 * @param  rot_cfg rotation configuration
 * @param  fg configuration of foreground layer that's to be rotated
 * @param  bg background layer configuration
 * @param  dst output layer configuration
 * @param  alpha alpha value for the image
 *
 * @retval HAL status
 */
static HAL_StatusTypeDef EPIC_ConfigRotation(EPIC_HandleTypeDef *epic,
        EPIC_TransformCfgTypeDef *rot_cfg,
        EPIC_BlendingDataType *fg,
        EPIC_BlendingDataType *bg,
        EPIC_BlendingDataType *dst,
        uint8_t alpha)
{
    HAL_StatusTypeDef ret;

    EPIC_TransformResultDef trans_result;

#ifndef SF32LB55X
    if (IS_FRAC_COORD_LAYER(fg) && IS_FRAC_COORD_LAYER(bg))
        RETURN_ERROR(epic, HAL_ERROR);//Support 1 fractional coordinate layer now
#endif /* SF32LB55X */

#ifdef HAL_EZIP_MODULE_ENABLED
    if (IS_EZIP_COLOR_MODE(fg->color_mode) && IS_EZIP_COLOR_MODE(bg->color_mode))
    {
        RETURN_ERROR(epic, HAL_ERROR);
    }

    if (IS_EZIP_COLOR_MODE(fg->color_mode) && (0 != rot_cfg->angle))
    {
        /* don't support ezip rotation */
        RETURN_ERROR(epic, HAL_ERROR);
    }
#endif

#if defined(SF32LB56X)||defined(SF32LB52X)
    if ((IS_YUV_COLOR_MODE(fg->color_mode) && IS_YUV_COLOR_MODE(bg->color_mode)) || IS_YUV_COLOR_MODE(dst->color_mode))
    {
        RETURN_ERROR(epic, HAL_ERROR);
    }

    if (IS_YUV_COLOR_MODE(fg->color_mode) && (0 != rot_cfg->angle))
    {
        /* don't support YUV rotation */
        RETURN_ERROR(epic, HAL_ERROR);
    }

#endif /* SF32LB56X */
    //rt_mutex_take(&epic_mutex, RT_WAITING_FOREVER);

    EPIC_TransResultInit(&trans_result, fg);

    EPIC_ClipLayerSrcByOutput(fg, rot_cfg, dst, &trans_result);
    EPIC_TransformVideoLayer(epic->Instance, EPIC_LAYER_IDX_VL, rot_cfg, fg, dst, &trans_result);
    EPIC_MakeAllLayerCoordValid(fg, bg, dst);

    EPIC_TransformResultDef trans_result_bg;
    EPIC_TransResultInit(&trans_result_bg, bg);
    EPIC_ClipLayerSrcByOutput(bg, NULL, dst, &trans_result_bg);

    EPIC_MakeAllLayerCoordValid(fg, bg, dst);

    if (EPIC_CalcIntersectArea(dst, fg, NULL))
    {
        if (IS_ALPHA_MASK_LAYER(fg))
        {
            ret = EPIC_ConfigMaskLayer(epic, EPIC_LAYER_IDX_0, fg);
        }
        else
        {
            EPIC_ConfigVideoLayer(epic, EPIC_LAYER_IDX_VL, &trans_result, fg, alpha, 1);
        }
    }

    /* configure layer0 for background */
    {

        /*
            Disable bg layer to accelerate blending if dst area is full covered by fg.
        */
        bool dst_coverd_by_fg = false;

        if ((alpha == EPIC_LAYER_OPAQUE) && (0 == rot_cfg->angle))
            if (IS_NO_ALPHA_COLOR_MODE(fg->color_mode))
                if ((fg->x_offset <= dst->x_offset)
                        && (fg->x_offset + fg->width >= dst->x_offset + dst->width)
                        && (fg->y_offset <= dst->y_offset)
                        && (fg->y_offset + fg->height >= dst->y_offset + dst->height)
                   )
                {
                    dst_coverd_by_fg = true;
                }



        if ((dst_coverd_by_fg) || (!EPIC_CalcIntersectArea(bg, dst, NULL)))
        {
            ret = EPIC_DisableLayer(epic->Instance, EPIC_LAYER_IDX_0);
        }
        else
        {
            ret = EPIC_ConfigLayer(epic, EPIC_LAYER_IDX_0, bg, true, 255);
            if (HAL_OK == ret)
            {
                ;
            }
            else
            {
                epic->ErrorCode = __LINE__;
            }
        }

    }

    /* configure ROI */
    ret = EPIC_ConfigOutputLayer(epic->Instance, fg, bg, dst);
    if (HAL_OK != ret)
    {
        epic->ErrorCode = __LINE__;
        return ret;
    }

#ifdef HAL_EZIP_MODULE_ENABLED
    if (IS_EZIP_COLOR_MODE(fg->color_mode) && EPIC_IsLayerActive(epic->Instance, EPIC_LAYER_IDX_VL))
    {
        ret = EPIC_ConfigEzipDec(epic, fg, dst, &trans_result);
    }
    else if (IS_EZIP_COLOR_MODE(bg->color_mode)  && EPIC_IsLayerActive(epic->Instance, EPIC_LAYER_IDX_0))
    {
        ret = EPIC_ConfigEzipDec(epic, bg, dst, &trans_result_bg);
    }
#endif

    //rt_mutex_release(&epic_mutex);

    /* update coord, bg/dst topleft is used as (0,0) */

    return ret;
}

/**
 * @brief  Start blending in polling mode
 *
 * @param  epic EPIC instance
 * @param  bg background layer configuration
 * @param  fg foreground layer configuration
 * @param  dst output layer configuration
 * @param  alpha alpha value
 *
 * @retval HAL status
 */
HAL_StatusTypeDef EPIC_ConfigBlend(EPIC_HandleTypeDef *epic,
                                   EPIC_BlendingDataType *bg,
                                   EPIC_BlendingDataType *fg,
                                   EPIC_BlendingDataType *dst,
                                   uint8_t alpha)
{
    HAL_StatusTypeDef ret;
    EPIC_LAYER_IDX fg_layer_idx = EPIC_LAYER_IDX_1;

    EPIC_TransformResultDef trans_result;

    EPIC_DEBUG_PRINT_LAYER_INFO(fg, "fg");
    EPIC_DEBUG_PRINT_LAYER_INFO(bg, "bg");
    EPIC_DEBUG_PRINT_LAYER_INFO(dst, "dst");

#ifndef SF32LB55X
    if (IS_ALPHA_MASK_LAYER(fg) && IS_ALPHA_MASK_LAYER(bg))
        RETURN_ERROR(epic, HAL_ERROR);//Only one mask layer supported.

    if (IS_FRAC_COORD_LAYER(fg) && IS_ALPHA_MASK_LAYER(fg))
        RETURN_ERROR(epic, HAL_ERROR);//Mask layer not support fractional coordinate

    if (IS_FRAC_COORD_LAYER(fg) && IS_FRAC_COORD_LAYER(bg))
        RETURN_ERROR(epic, HAL_ERROR);//Support 1 fractional coordinate layer now

#endif /* SF32LB55X */

    EPIC_TransResultInit(&trans_result, fg);

#ifdef HAL_EZIP_MODULE_ENABLED
    if (IS_EZIP_COLOR_MODE(bg->color_mode) && IS_EZIP_COLOR_MODE(fg->color_mode))
    {
        RETURN_ERROR(epic, HAL_ERROR);
    }
#endif

#if defined(SF32LB56X)||defined(SF32LB52X)
    if ((IS_YUV_COLOR_MODE(fg->color_mode) && IS_YUV_COLOR_MODE(bg->color_mode)) || IS_YUV_COLOR_MODE(dst->color_mode))
    {
        RETURN_ERROR(epic, HAL_ERROR);
    }
#endif /* SF32LB56X */

#ifndef SF32LB55X
    if (IS_FRAC_COORD_LAYER(fg))
    {
        EPIC_TransformCfgTypeDef rot_cfg;
        HAL_EPIC_RotDataInit(&rot_cfg);
        EPIC_ClipLayerSrcByOutput(fg, &rot_cfg, dst, &trans_result);
        //Use video layer if 'fg' has fractional coordinate
        ret = EPIC_TransformVideoLayer(epic->Instance, EPIC_LAYER_IDX_VL, &rot_cfg, fg, dst, &trans_result);
        EPIC_MakeAllLayerCoordValid(fg, bg, dst);

    }
#endif /* SF32LB55X */

    EPIC_TransformResultDef trans_result_bg;
    EPIC_TransResultInit(&trans_result_bg, bg);
    EPIC_ClipLayerSrcByOutput(bg, NULL, dst, &trans_result_bg);

    EPIC_MakeAllLayerCoordValid(fg, bg, dst);

    {
        bool dst_in_fg; //dst area in fg area


        if ((fg->x_offset <= dst->x_offset)
                && (fg->x_offset + fg->width >= dst->x_offset + dst->width)
                && (fg->y_offset <= dst->y_offset)
                && (fg->y_offset + fg->height >= dst->y_offset + dst->height)
           )
            dst_in_fg = true;
        else
            dst_in_fg = false;


        /*
            If fg full cover bg, just disable bg layer to speed up
        */
        if ((IS_NO_ALPHA_COLOR_MODE(fg->color_mode) && (alpha == EPIC_LAYER_OPAQUE) && dst_in_fg)
                || (!EPIC_CalcIntersectArea(bg, dst, NULL)))
        {
            ret = EPIC_DisableLayer(epic->Instance, EPIC_LAYER_IDX_0);
        }
        else
        {
            ret = EPIC_ConfigLayer(epic, EPIC_LAYER_IDX_0, bg, true, 255);
        }
    }
    if (HAL_OK != ret)
    {
        return ret;
    }

    if (IS_ALPHA_MASK_LAYER(fg))
    {
        ret = EPIC_ConfigMaskLayer(epic, EPIC_LAYER_IDX_0, fg);
    }
    else if ((!EPIC_CalcIntersectArea(fg, dst, NULL)))
    {
        ret = HAL_OK;//Not enable fg
    }
#ifndef SF32LB55X
    else if (IS_FRAC_COORD_LAYER(fg))
    {
        //Use video layer if 'fg' has fractional coordinate
        ret = EPIC_ConfigVideoLayer(epic, EPIC_LAYER_IDX_VL, &trans_result, fg, alpha, 1);
        fg_layer_idx = EPIC_LAYER_IDX_VL;
    }
#endif /* SF32LB55X */
    else
    {
        ret = EPIC_ConfigLayer(epic, EPIC_LAYER_IDX_1, fg, true, alpha);
    }

    if (HAL_OK != ret)
    {
        RETURN_ERROR(epic, ret);
    }
    ret = EPIC_ConfigOutputLayer(epic->Instance, fg, bg, dst);
    if (HAL_OK != ret)
    {
        RETURN_ERROR(epic, ret);
    }
#ifdef HAL_EZIP_MODULE_ENABLED
    if (IS_EZIP_COLOR_MODE(fg->color_mode) && EPIC_IsLayerActive(epic->Instance, fg_layer_idx))
    {
        ret = EPIC_ConfigEzipDec(epic, fg, dst, &trans_result);
    }
    else if (IS_EZIP_COLOR_MODE(bg->color_mode) && EPIC_IsLayerActive(epic->Instance, EPIC_LAYER_IDX_0))
    {
        ret = EPIC_ConfigEzipDec(epic, bg, dst, &trans_result_bg);
    }
#endif

    return HAL_OK;
}



/**
 * @brief Allocate appropriate EPIC layer for each input layer
 * @param input           -  [IN] input layers
 * @param input_layer_num -  [IN] input layer num
 * @param transform_cfg   -  [IN] layer's transform param
 * @param epic_layer      -  [OUT] EPIC layer idx
 * @param vl_depth        -  [OUT] Video layer blend depth
 * @return HAL_ERROR if can't allocate EPIC layer for every input layer, else return HAL_OK
 */
static HAL_StatusTypeDef EPIC_ChooseLayer(EPIC_BlendingDataType *input,
        EPIC_TransformCfgTypeDef *transform_cfg,
        uint8_t input_layer_num,
        EPIC_LAYER_IDX *epic_layer,
        uint8_t *vl_depth)
{
    int8_t cur_fix_depth_layer;
    uint8_t video_layer_valid = 1;
    uint8_t ezip_num = 0;
#if defined(SF32LB56X)||defined(SF32LB52X)
    uint8_t yuv_num = 0;
#endif /* SF32LB56X */
#ifndef SF32LB55X
    uint8_t mask_num = 0;
#endif /* SF32LB55X */

#if defined(SF32LB52X)
    cur_fix_depth_layer = (int8_t)EPIC_LAYER_IDX_0;
#else
    cur_fix_depth_layer = (int8_t)EPIC_LAYER_IDX_2;
#endif
    *vl_depth = 0xFF;

    if (0 == input_layer_num) return HAL_OK;

    for (int8_t i = input_layer_num - 1; i >= 0 ; i--)
    {
        HAL_ASSERT(input[i].total_width >= input[i].width);
        if (IS_NEED_TRANSFROM_LAYER(&input[i], (&transform_cfg[i]))) //Use video layer
        {
            if (IS_ALPHA_MASK_LAYER(&input[i]))
            {
                return HAL_ERROR; //Mask layer not support transform
            }
#if defined(SF32LB58X)
            else if ((int8_t)EPIC_LAYER_IDX_2 == cur_fix_depth_layer) //L2 support transfrom
            {
                epic_layer[i]       = cur_fix_depth_layer;
                cur_fix_depth_layer--;
            }
#endif /* SF32LB55X */
            else if (video_layer_valid)
            {
                epic_layer[i] = EPIC_LAYER_IDX_VL;
                video_layer_valid = 0;
                *vl_depth = cur_fix_depth_layer + 1;
            }
            else
                return HAL_ERROR;
        }
        else if (IS_ALPHA_MASK_LAYER(&input[i]))
        {
#ifndef SF32LB55X
            //Only one mask layer supported.
            if (++mask_num > 1)  return HAL_ERROR;
            //Mask layer should not at bottom
            if (0 == i)  return HAL_ERROR;

            epic_layer[i] = EPIC_SPECIAL_LAYER_MASK;
#else
            return HAL_ERROR;
#endif /* SF32LB55X */
        }
        else
        {
            if ((video_layer_valid) &&
                    ((cur_fix_depth_layer < (int8_t)EPIC_LAYER_IDX_0)

                     /*VL has independent read bus for low speed memory,
                       and last layer always is framebuffer with no transform requirement
                       */
                     || (1 == i)
                    ))
            {
                epic_layer[i] = EPIC_LAYER_IDX_VL;
                video_layer_valid = 0;
                *vl_depth = cur_fix_depth_layer + 1;
            }
            else if (cur_fix_depth_layer >= (int8_t)EPIC_LAYER_IDX_0)
            {
                epic_layer[i]       = cur_fix_depth_layer;
                cur_fix_depth_layer--;
            }
            else
            {
                return HAL_ERROR; //No avaliable layer
            }
        }

        if (IS_EZIP_COLOR_MODE(input[i].color_mode))
        {
#ifdef HAL_EZIP_MODULE_ENABLED
            if (++ezip_num > 1) return HAL_ERROR;
#else
            return HAL_ERROR;
#endif
        }

        if (IS_YUV_COLOR_MODE(input[i].color_mode))
        {
#if defined(SF32LB56X)||defined(SF32LB52X)
            if (++yuv_num > 1) return HAL_ERROR;
#else
            return HAL_ERROR;
#endif /* SF32LB56X */
        }
    }

    return HAL_OK;
}


/**
 * @brief
 * @param epic -
 * @param input - layers's blending data
 * @param alpha - layers's alpha
 * @param transform_cfg - layers's transfrom configuration
 * @param layer_num - layers number
 * @return
 */
static HAL_StatusTypeDef EPIC_ConfigBlendEx(EPIC_HandleTypeDef *epic,
        EPIC_BlendingDataType *input,
        uint8_t *alpha,
        EPIC_TransformCfgTypeDef *transform_cfg, uint8_t layer_num)
{
    HAL_StatusTypeDef ret;
    uint32_t i;
    uint8_t video_layer_depth;

    EPIC_LAYER_IDX epic_layer[MAX_EPIC_LAYER];
    EPIC_TransformResultDef trans_result[MAX_EPIC_LAYER];

    uint32_t ezip_layer_idx = UINT32_MAX;

    uint8_t input_layer_num = layer_num - 1;
    EPIC_BlendingDataType *output = &input[layer_num - 1];
#ifdef EPIC_SUPPORT_DITHER
    uint32_t output_bit_mask, input_bit_mask = 0;
#endif /* EPIC_SUPPORT_DITHER */

    EPIC_PRINTF("\r\n EPIC_ConfigBlendEx \r\n");
    //1.Choose EPIC layer for each input
    ret = EPIC_ChooseLayer(input, transform_cfg, input_layer_num, epic_layer, &video_layer_depth);
    if (HAL_OK != ret)
    {
        return ret;
    }

    //2.Transfrom layer to update other layer's coordinates
    for (i = 0; i < input_layer_num; i++)
    {
        EPIC_TransResultInit(&trans_result[i], &input[i]);

        EPIC_PRINTF("%d: EPIC_layer idx %d \n", i, epic_layer[i]);

        EPIC_ClipLayerSrcByOutput(&input[i], &transform_cfg[i], output, &trans_result[i]);
        if (IS_NEED_TRANSFROM_LAYER(&input[i], &transform_cfg[i]))
        {
            ret = EPIC_TransformVideoLayer(epic->Instance, epic_layer[i],
                                           &transform_cfg[i], &input[i], output, &trans_result[i]);
            if (HAL_OK != ret)
            {
                return ret;
            }
            if (!EPIC_CalcIntersectArea(&input[i], output, NULL))
            {
                input[i].width  = 0;//Inactive this layer.
                input[i].height = 0;
            }

        }
        EPIC_MakeAllLayerCoordValidEx(input, input_layer_num + 1); //+1 for output layer
    }

    //3.Config EPIC layer
    for (i = 0; i < input_layer_num; i++)
    {
        if (!EPIC_CalcIntersectArea(&input[i], output, NULL))
        {
            //No intersect skip layer
            continue;
        }
        else if (IS_NEED_TRANSFROM_LAYER(&input[i], &transform_cfg[i])
                 || (EPIC_LAYER_IDX_VL == epic_layer[i])
#ifndef SF32LB55X
                 || (EPIC_LAYER_IDX_2 == epic_layer[i])
#endif /* SF32LB55X */
                )
        {
            ret = EPIC_ConfigVideoLayer(epic, epic_layer[i], &trans_result[i], &input[i], alpha[i], video_layer_depth);
        }
        else if (IS_ALPHA_MASK_LAYER(&input[i]))
        {
            HAL_ASSERT(i > 0);
            //Mask next layer
            ret = EPIC_ConfigMaskLayer(epic, epic_layer[i - 1], &input[i]);
        }
        else
        {
            ret = EPIC_ConfigLayer(epic, epic_layer[i], &input[i], true, alpha[i]);
        }
#ifdef EPIC_SUPPORT_DITHER
        if (!IS_EZIP_COLOR_MODE(input[i].color_mode))
        {
            input_bit_mask |= EPIC_GetColorBitMask(input[i].color_mode, input[i].ax_mode);
        }
#endif /* EPIC_SUPPORT_DITHER */

        if (HAL_OK != ret)
        {
            return ret;
        }


        if (IS_EZIP_COLOR_MODE(input[i].color_mode))
        {
            ezip_layer_idx = i;
        }
    }

    ret = EPIC_ConfigOutputLayerEx(epic, input, input_layer_num, output);
    if (HAL_OK != ret)
    {
        return ret;
    }

#ifdef EPIC_SUPPORT_DITHER
    output_bit_mask = EPIC_GetColorBitMask(output->color_mode, ALPHA_BLEND_INVALID);
#endif /* EPIC_SUPPORT_DITHER */
#ifdef HAL_EZIP_MODULE_ENABLED
    if (ezip_layer_idx != UINT32_MAX)
    {
        ret = EPIC_ConfigEzipDec(epic, &input[ezip_layer_idx], output, &trans_result[ezip_layer_idx]);
    }
#endif
#ifdef EPIC_SUPPORT_DITHER
    EPIC_ConfigDither(epic, (input_bit_mask & (~output_bit_mask)));
#endif /* EPIC_SUPPORT_DITHER */

    return ret;
}



HAL_StatusTypeDef EPIC_ConfigFilling(EPIC_HandleTypeDef *epic, EPIC_FillingCfgTypeDef *param)
{
    EPIC_BlendingDataType blending_cfg;
    HAL_StatusTypeDef ret;


    HAL_EPIC_BlendDataInit(&blending_cfg);
    blending_cfg.data = param->start;
    blending_cfg.color_mode = param->color_mode;
    blending_cfg.width = param->width;
    blending_cfg.height = param->height;
    blending_cfg.total_width = param->total_width;

    if (EPIC_LAYER_OPAQUE != param->alpha)
    {
        EPIC_TransformResultDef trans_result;
        EPIC_TransResultInit(&trans_result, &blending_cfg);

        EPIC_ConfigVideoLayer(epic, EPIC_LAYER_IDX_VL, &trans_result, &blending_cfg,
                              (0 == param->alpha) ? 255 : (256 - param->alpha), 1); //Issue 2090
    }

    blending_cfg.color_en = true;
    blending_cfg.color_r = param->color_r;
    blending_cfg.color_g = param->color_g;
    blending_cfg.color_b = param->color_b;
    ret = EPIC_ConfigOutputLayer(epic->Instance, &blending_cfg, &blending_cfg, &blending_cfg);
    if (HAL_OK != ret)
    {
        epic->ErrorCode = __LINE__;
        goto __EXIT;
    }

    epic->Instance->CANVAS_BG = MAKE_REG_VAL(param->color_r, EPIC_CANVAS_BG_RED_Msk, EPIC_CANVAS_BG_RED_Pos)
                                | MAKE_REG_VAL(param->color_g, EPIC_CANVAS_BG_GREEN_Msk, EPIC_CANVAS_BG_GREEN_Pos)
                                | MAKE_REG_VAL(param->color_b, EPIC_CANVAS_BG_BLUE_Msk, EPIC_CANVAS_BG_BLUE_Pos);

__EXIT:
    return ret;
}


static HAL_StatusTypeDef EPIC_ConfigGrad(EPIC_HandleTypeDef *epic, EPIC_GradCfgTypeDef *param)
{
    EPIC_TransformCfgTypeDef grd_cfg;
    EPIC_BlendingDataType output_cfg, fg_cfg;
    int16_t col, row;
    HAL_StatusTypeDef ret;
    static EPIC_ColorDef fg_data[3][3];
    EPIC_TransformResultDef trans_result;

    memset(&fg_cfg, 0, sizeof(EPIC_BlendingDataType));
    memset(&output_cfg, 0, sizeof(EPIC_BlendingDataType));
    memset(&grd_cfg, 0, sizeof(EPIC_TransformCfgTypeDef));

    output_cfg.data = param->start;
    output_cfg.color_mode = param->color_mode;
    output_cfg.width = param->width;
    output_cfg.height = param->height;
    output_cfg.x_offset = 0;
    output_cfg.y_offset = 0;
    output_cfg.total_width = param->total_width;
    output_cfg.color_en = false;


    if (output_cfg.width >= 3)
    {
        fg_cfg.total_width = 3;
        fg_cfg.width = 3;

        grd_cfg.scale_x = (EPIC_INPUT_SCALING_FACTOR_1 * 2) / output_cfg.width;


        for (row = 0; row < 2; row++)
        {
            fg_data[row * 2][0] = param->color[row][0];
            fg_data[row * 2][2] = param->color[row][1];
            EPIC_MixColor(&fg_data[row * 2][0], &fg_data[row * 2][2], &fg_data[row * 2][1]);
        }
    }
    else if (2 == output_cfg.width)
    {
        fg_cfg.total_width = 2;
        fg_cfg.width = 2;
        grd_cfg.scale_x = EPIC_INPUT_SCALING_FACTOR_1;
        for (row = 0; row < 2; row++)
        {
            fg_data[row * 2][0] = param->color[row][0];
            fg_data[row * 2][2] = param->color[row][1];
        }
    }
    else if (1 == output_cfg.width)
    {
        fg_cfg.total_width = 1;
        fg_cfg.width = 1;
        grd_cfg.scale_x = EPIC_INPUT_SCALING_FACTOR_1;
        for (row = 0; row < 2; row++)
        {
            EPIC_MixColor(&fg_data[row * 2][0], &param->color[row][0], &param->color[row][1]);
        }
    }
    else
    {
        HAL_ASSERT(output_cfg.width > 0);
    }

    if (output_cfg.height >= 3)
    {
        fg_cfg.height = 3;
        grd_cfg.scale_y = (EPIC_INPUT_SCALING_FACTOR_1 * 2) / output_cfg.height;

        for (col = 0; col < fg_cfg.width; col++)
        {
            EPIC_MixColor(&fg_data[0][col], &fg_data[2][col], &fg_data[1][col]);
        }
    }
    else if (2 == output_cfg.height)
    {
        fg_cfg.height = 2;
        grd_cfg.scale_y = EPIC_INPUT_SCALING_FACTOR_1;
        for (col = 0; col < fg_cfg.width; col++)
        {
            fg_data[1][col] = fg_data[2][col];
        }

    }
    else if (1 == output_cfg.height)
    {
        fg_cfg.height = 1;
        grd_cfg.scale_y = EPIC_INPUT_SCALING_FACTOR_1;
        for (col = 0; col < fg_cfg.width; col++)
        {
            EPIC_MixColor(&fg_data[0][col], &fg_data[2][col], &fg_data[0][col]);
        }
    }
    else
    {
        HAL_ASSERT(output_cfg.height > 0);
    }


    fg_cfg.data = (uint8_t *)&fg_data[0][0];
    fg_cfg.color_mode = EPIC_COLOR_ARGB8888;
    fg_cfg.x_offset = 0;
    fg_cfg.y_offset = 0;
    fg_cfg.color_en = false;

    EPIC_TransResultInit(&trans_result, &fg_cfg);

    ret = EPIC_TransformVideoLayer(epic->Instance, EPIC_LAYER_IDX_VL, &grd_cfg, &fg_cfg, &output_cfg, &trans_result);

    if (HAL_OK != ret)
    {
        goto __EXIT;
    }

    if (EPIC_CalcIntersectArea(&fg_cfg, &output_cfg, NULL))
    {
        ret = EPIC_ConfigVideoLayer(epic, EPIC_LAYER_IDX_VL, &trans_result, &fg_cfg, EPIC_LAYER_OPAQUE, 1);
        if (HAL_OK != ret)
        {
            goto __EXIT;
        }

        ret = EPIC_ConfigLayer(epic, EPIC_LAYER_IDX_0, &output_cfg, true, 255);
        if (HAL_OK != ret)
        {
            goto __EXIT;
        }
    }


    ret = EPIC_ConfigOutputLayer(epic->Instance, &output_cfg, &output_cfg, &output_cfg);


__EXIT:
    return ret;
}




#endif /* HAL_EPIC_MODULE_ENABLED */

/**
  * @} EPIC
  */

/**
  * @} BF0_HAL_Driver
  */

/**
  * @}
  */