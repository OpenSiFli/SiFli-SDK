/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file drv_epic_private.h
 *
 */
#include "rtthread.h"

#ifndef DRV_EPIC_PRIVATE_H
#define DRV_EPIC_PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef DBG_LEVEL
#undef DBG_LEVEL
#define  DBG_LEVEL            DBG_INFO  //DBG_LOG //
#endif
#ifdef LOG_TAG
#undef LOG_TAG
#define LOG_TAG                "drv.epic"
#endif
#include "log.h"

#ifndef __DEBUG__
#define __DEBUG__ 1

#endif


#ifndef MIN
#define MIN(x,y) (((x)<(y))?(x):(y))
#endif

#ifndef MAX
#define MAX(x,y) (((x)>(y))?(x):(y))
#endif

#ifndef ABS
#define ABS(x)          ((x) < (0) ? -(x) : (x))
#endif

#define EPIC_OPA_MIN 2
#define EPIC_OPA_MAX 255

#undef OPA_MAX
#define OPA_MAX 254


#define IS_DCACHED_RAM(addr) (((uint32_t) addr) >= (PSRAM_BASE))
#define GPU_BLEND_EXP_MS     5000





#ifdef DRV_EPIC_NEW_API

#define render_list_pool_max    2
#define letter_pool_max         800//800
#define mask_buf_max_bytes      (16*1024)
#define mask_buf2_max_bytes     1600
#if defined(ROTATE_BUF_SIZE) && defined(ROTATE_BUF_IN_SRAM)
#define rotate_buf_bytes   ROTATE_BUF_SIZE
#else
#define rotate_buf_bytes   0
#endif

#define rl_flag_commit          0x01
#define rl_flag_rendering       0x02

#define debug_rl_hist_num       8 //Set '0' to disable history

#define radius_circle           0x7FFF /**< A very big radius to always draw as circle*/
#define max_flash_num           4
typedef enum
{
    HAL_API_BLEND_EX,
    HAL_API_CONT_BLEND,
    HAL_API_CONT_BLEND_STOP,
    HAL_API_CONT_BLEND_ASYNC_STOP,
    HAL_API_TRANSFORM,
    HAL_API_COPY,
    HAL_API_ALL_STOP,
} HAL_API_TypeDef;

typedef struct
{
    uint32_t  used;
    uint32_t  flag;
    EPIC_LayerConfigTypeDef dst;

    uint16_t src_list_alloc_len; /* Allocated src list len */
    uint16_t src_list_len; /* Committed src len*/
    rt_list_t src_list;    /* Committed source list*/
    drv_epic_letter_type_t *letter_pool;
    uint32_t letter_pool_free;  /* The size of the letter pool that has been used*/
    uint32_t letter_pool_size;  /* The maximum size of the letter pool for this render list*/
    uint8_t  letter_pool_index; /* The index of the letter pool from stack*/
    EPIC_AreaTypeDef  commit_area; /*Committed invalid area*/

} priv_render_list_t;


typedef struct
{
    priv_render_list_t *rl;
    uint16_t  src_idx;
    drv_epic_operation *operation;
    uint32_t  start_tick;
    uint32_t  cost_us;
} priv_render_hist_t;


typedef struct
{
    priv_render_list_t *rl;
} epic_cbk_ctx_t;

typedef struct
{
    uint32_t sw; /*software time, us*/
    uint32_t hw;/*EPIC execute time, us*/
    uint32_t async_wait; /*Wait previous time, us*/
    uint32_t sync_wait; /*Wait previous time, us*/
} operation_detail_ctx_t;

typedef struct
{
    uint32_t is_grad;
    uint32_t argb8888;
    rect_vertex_color_t *p_grad_color;
    const EPIC_AreaTypeDef *p_rect_area;
} draw_rect_dsc_t;
#endif /*DRV_EPIC_NEW_API*/

typedef struct _split_render_t
{
    drv_epic_op_type_t  op;
    EPIC_AreaTypeDef dst_area;
    const uint8_t *dst_data;
    EPIC_AreaTypeDef render_area;       //Part of 'dst_area'
    EPIC_AreaTypeDef next_render_area;  //Part of 'render_area'
} split_render_t;

typedef struct
{
    EPIC_HandleTypeDef epic_handle;

    EPIC_TypeDef RamEPIC;
#ifdef HAL_EZIP_MODULE_ENABLED
    EZIP_TypeDef RamEZIP;
    EZIP_HandleTypeDef ezip_handle;
#endif /* HAL_EZIP_MODULE_ENABLED */
#ifdef HAL_JPEGD_MODULE_ENABLED
    JPEGD_TypeDef RamJPEGD;
    JPEGD_HandleTypeDef jpegd_handle;
#endif /* HAL_JPEGD_MODULE_ENABLED */


#ifdef EPIC_DEBUG
    EPIC_OpHistTypeDef epic_op_hist;
#endif

    uint32_t hclk_freq_Mhz;
    uint32_t gpu_timeout_cnt;

#ifndef DRV_EPIC_NEW_API
    EPIC_LayerConfigTypeDef input_layers[MAX_EPIC_LAYER];
    uint8_t    input_layer_cnt;
    EPIC_LayerConfigTypeDef output_layer;
    EPIC_ColorDef grad_color[2][2];

    split_render_t split_rd;
    drv_epic_cplt_cbk cbk;

    bool cont_mode;

    uint32_t gpu_last_op;
    uint32_t gpu_fg_addr;
    uint32_t gpu_bg_addr;
    uint32_t gpu_mask_addr;
    uint32_t gpu_output_addr;
    uint32_t gpu_output_size;
#else /*DRV_EPIC_NEW_API*/

    EPIC_HandleTypeDef epic_handle2; //Shadow handle
#ifdef HAL_EZIP_MODULE_ENABLED
    EZIP_HandleTypeDef ezip_handle2; //Shadow handle
#endif
#ifdef HAL_JPEGD_MODULE_ENABLED
    JPEGD_HandleTypeDef jpegd_handle2; //Shadow handle
#endif /* HAL_JPEGD_MODULE_ENABLED */
    epic_cbk_ctx_t  epic_cb_ctx;
    priv_render_list_t *render_list_pool;
    priv_render_list_t *using_rl_stack[render_list_pool_max]; //Committing rl stack
    uint32_t using_rl_count; //Committing rl stack depth
#if debug_rl_hist_num > 0
    uint32_t           hist_idx;
    priv_render_hist_t hist[debug_rl_hist_num];
#endif /* debug_rl_hist_num > 0 */

    uint8_t *buf1; //Fast ping-ping render buffer 1
    uint8_t *buf2; //Fast ping-ping render buffer 2
    uint32_t buf_bytes; //Fast ping-ping render buffer size
    uint8_t *cur_buf; //Current using ping-ping render

    uint8_t *mask_buf_pool; //Pool for draw mask
    uint8_t *mask_buf2_pool; //Pool2 for circle mask
    uint8_t *rotate_buf;

    EPIC_HandleTypeDef *using_epic;

    uint32_t flash_addr[max_flash_num]; //Current rl accessed flash

    /*statistics*/
    //Global statistics
    uint32_t last_statistics_ms; /* Last statistics time*/
    uint32_t start_ms;   /*rd start ms*/
    uint32_t start_epic_wait_cnt;
    uint32_t start_epic_cnt;
    uint32_t start_hal_epic_cnt;
    uint32_t rd_count; /* render frame counter*/
    uint32_t rd_min;   /* render frame minimal time */
    uint32_t rd_max;  /* render frame maximum time */
    uint32_t rd_total; /*ms, render task total running time*/
    uint32_t rd_usr_cb_total; /*ms*/
    uint32_t rd_epic_hw_us;
    uint32_t rd_epic_hal_us;
    uint32_t rd_epic_sync_wait_us;
    uint32_t rd_epic_async_wait; /*ms*/
    uint32_t letter_buf_pool_max;  /* The maximum size of the letter pool*/
    uint32_t letter_pool_used_max; /* The maximum size of the letter pool that has been used*/

    //Detail statistics
    operation_detail_ctx_t rd_operations_detail[DRV_EPIC_DRAW_MAX];
    operation_detail_ctx_t *p_last_rd_operation;
    uint32_t last_rd_operation_start_epic_cnt;



    uint8_t task_idle; //1: task is idle, 0: task is busy
    drv_epic_rotate_t rotated; //Whether rotation is supported

    uint32_t dbg_flag_dis_ram_instance: 1;
    uint32_t dbg_flag_print_rl : 1;//Show render list before start
    uint32_t dbg_flag_print_exe_detail : 1; //Show render operation execute detail
    uint32_t dbg_flag_print_statistics: 1; //Show render statistics
    uint32_t dbg_flag_reserved: 28;
    uint32_t dbg_flag_dis_merge_operations;
    uint32_t dbg_flag_dis_operations;
    uint32_t dbg_src_addr;
    uint32_t dbg_mask_buf_pool_max;
    uint32_t dbg_render_buf_max;


    uint32_t magic_num; /* Magic number to check whether the handle is valid,the vaule should be 0xBEEFBEEF*/

    /* Put OS related variables after magic_num to avoid different OS data structure between HCPU&ACPU*/
    struct rt_semaphore render_sema;
    struct rt_semaphore rl_sema;
    struct rt_thread task;
    rt_mq_t  mq;
#endif /* DRV_EPIC_NEW_API */
} EPIC_DrvTypeDef;

#define EPIC_DRV_MAGIC_NUM 0xBEEFBEEF
/*
    Private Macros
*/

#if defined(DRV_EPIC_ALLOC_USE_ANIM_HEAP)
#include "app_mem.h"
#define epic_malloc          app_anim_alloc
#define epic_realloc         app_anim_realloc
#define epic_calloc          app_anim_calloc
#define epic_free            app_anim_free
#elif defined(SOLUTION)
#include "app_mem.h"
#define epic_malloc          app_malloc
#define epic_realloc         app_realloc
#define epic_calloc          app_calloc
#define epic_free            app_free
#else
#define epic_malloc          rt_malloc
#define epic_realloc         rt_realloc
#define epic_calloc          rt_calloc
#define epic_free            rt_free
#endif

#ifdef _MSC_VER
#define RETURN_ADDR ((rt_uint32_t) _ReturnAddress())
#else
#define RETURN_ADDR ((rt_uint32_t) __builtin_return_address(0))
#endif

#define AreaString "x0y0x1y1=[%d,%d,%d,%d]"

#define FORMATED_LAYER_INFO(layer, layer_name) \
                        "%s,cf=0x%x,data=%x,total_w=%d,"AreaString" frac[%x,%x]", \
                                    (layer_name), (layer)->color_mode, (layer)->data, (layer)->total_width,\
                                    (layer)->x_offset, \
                                    (layer)->y_offset, \
                                    (layer)->x_offset + (layer)->width - 1, \
                                    (layer)->y_offset + (layer)->height - 1,\
                                    (layer)->x_offset_frac, (layer)->y_offset_frac

#define FORMATED_LAYER_EXTRA_INFO(layer) \
                        "ax=%d[0:NONE,1:MASK], alpha=%d, angle=%d, scale_xy=%d,%d, pivot_xy=%d,%d  mirror_hv=%d,%d", \
                                    (layer)->ax_mode, (layer)->alpha, \
                                    (layer)->transform_cfg.angle, \
                                    (layer)->transform_cfg.scale_x, (layer)->transform_cfg.scale_y, \
                                    (layer)->transform_cfg.pivot_x, (layer)->transform_cfg.pivot_y, \
                                    (layer)->transform_cfg.h_mirror, (layer)->transform_cfg.v_mirror


#define AreaParams(area) (area)->x0,(area)->y0,(area)->x1,(area)->y1
#define PRINT_AREA_INFO(area, area_name) LOG_D("%s "AreaString, area_name, AreaParams(area))
#define PRINT_LAYER_INFO(layer, layer_name)  LOG_D(FORMATED_LAYER_INFO(layer, layer_name))
#define PRINT_LAYER_EXTRA_INFO(layer)    LOG_D(FORMATED_LAYER_EXTRA_INFO(layer))

#define _get_layer_area(p_area, p_layer) \
    (p_area)->x0 = (p_layer)->x_offset; \
    (p_area)->y0 = (p_layer)->y_offset; \
    (p_area)->x1 = (p_layer)->x_offset + (p_layer)->width - 1; \
    (p_area)->y1 = (p_layer)->y_offset + (p_layer)->height - 1; \

#ifdef DRV_EPIC_NEW_API_DUAL_CORE_ACPU
#define DRV_EPIC_ASSERT(expr) RT_ASSERT(expr)
#else
#define DRV_EPIC_ASSERT(expr) do{\
    if(!(expr)){print_gpu_error_info();\
    RT_ASSERT(expr);}\
}while(0)
#endif

#if OUTPUT_TO_GPIO_PIN
extern void BSP_TEST_GPIO_SET(uint32_t pin, uint32_t v);

#define __DEBUG_RENDER_LOCK__(op)      do{if (DRV_EPIC_DRAW_FILL == (ops)){\
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

/*
    External Variables
*/
extern EPIC_DrvTypeDef *gp_drv_epic;

/*
    External Functions
*/

void print_gpu_error_info(void);
bool drv_gpu_is_busy(void);

#ifdef DRV_EPIC_NEW_API
rt_err_t drv_epic_render_list_init(void);
void drv_epic_ll_open(EPIC_DrvTypeDef *p_drv_epic);
char *operation_name(drv_epic_op_type_t op);
void print_operation(const char *name, const drv_epic_operation *op);
uint32_t GetElapsedUs(uint32_t prev_tick, uint32_t cur_tick);

rt_err_t drv_epic_render_list(void *p_drv_epic, void *list);
rt_err_t drv_epic_render_list_scale(void *p_drv_epic, void *list, void *p_scaled_area);
HAL_StatusTypeDef Call_Hal_Api(HAL_API_TypeDef api, void *p1, void *p2, void *p3);

#else
void drv_epic_single_init(void);
void drv_epic_single_open(EPIC_DrvTypeDef *p_drv_epic);
#endif


/*
    Internal Functions
*/
static inline uint8_t *get_map_ptr_by_xy(const uint8_t *map, uint8_t pixel_depth, uint32_t total_width, int32_t x, int32_t y)
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

static inline uint32_t get_layer_size(EPIC_BlendingDataType *p_layer)
{
    uint32_t pixel_size = HAL_EPIC_GetColorDepth(p_layer->color_mode);

    return ((pixel_size * p_layer->total_width * p_layer->height) + 7) >> 3;
}

static inline void clip_layer_to_area(EPIC_BlendingDataType *p_layer,
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

static inline void clip_intersect_area(EPIC_AreaTypeDef *src_area, EPIC_AreaTypeDef *dst_area)
{
    src_area->x0 = HAL_MAX(src_area->x0, dst_area->x0);
    src_area->x1 = HAL_MIN(src_area->x1, dst_area->x1);
    src_area->y0 = HAL_MAX(src_area->y0, dst_area->y0);
    src_area->y1 = HAL_MIN(src_area->y1, dst_area->y1);
}




#ifdef __cplusplus
}
#endif
#endif /* DRV_EPIC_PRIVATE_H */