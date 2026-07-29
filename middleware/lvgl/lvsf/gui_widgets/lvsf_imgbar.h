/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LVSF_IMGBAR_H_
#define _LVSF_IMGBAR_H_

#ifdef __cplusplus
extern "C" {
#endif


/*********************
 *      INCLUDES
 *********************/
#include "lvsf_conf_internal.h"
#include "lvsf_obj_ext.h"

#if LVSF_USE_IMGBAR!=0

/*Testing of dependencies*/

#if LV_USE_IMG == 0
#error "lv_imgbar: lv_img is required. Enable it in lv_conf.h (LV_USE_IMG  1) "
#endif
/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**
 * @brief 图像条处理回调函数类型
 * @param obj 图像条对象指针
 * @param percent 当前百分比
 * @note 当图像条需要处理进度变化时调用该回调函数
 */
typedef void (*lv_imgbar_process_cb_t)(struct _lv_obj_t *obj, uint8_t percent);

/**
 * @brief 图像条方向枚举
 */
enum
{
    BAR_DIR_LEFT_TO_RIGTH,      /**<从左到右*/
    BAR_DIR_RIGTH_TO_LEFT,      /**<从右到左*/
    BAR_DIR_TOP_TO_BOTTOM,      /**<从上到下*/
    BAR_DIR_BOTTOM_TO_TOP,      /**<从下到上*/
};
typedef uint8_t lv_imgbar_dir_t;

/**
 * @brief 图像条模式枚举
 */
enum
{
    IMG_BAR_MODE_BAR,           /**<条形模式*/
    IMG_BAR_MODE_SWITCH,        /**<开关模式*/
};
typedef uint8_t lv_imgbar_mode_t;

/**
 * @brief 图像条控件数据结构
 */
typedef struct
{
    /*Ext. of ancestor*/
    lv_obj_t bg;

    uint8_t drag : 1;      /**<激活拖动和释放修改效果*/
    uint8_t anim : 1;      /**<进度变化时显示动画*/
    uint8_t mode : 2;      /**<lv_imgbar_mode_t*/
    uint8_t dir : 3;       /**<lv_imgbar_dir_t*/
    uint8_t stop_timer : 1;
    uint8_t percent;   /**<当前显示的进度*/
    uint16_t size;     /**<当前显示的大小*/

    lv_obj_t *img_bg;           /**<背景图像*/
    lv_obj_t *img_fg;           /**<前景图像*/
    lv_obj_t *img_indicator;    /**<指示器图像*/
    lv_obj_t *obj_bg;           /**<背景对象*/

    lv_coord_t offset_x;        /**<X偏移*/
    lv_coord_t offset_y;        /**<Y偏移*/
    lv_imgbar_process_cb_t user_cb; /**<用户回调函数*/
} lv_imgbar_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * @brief 图像条刷新定时器回调函数
 * @param timer 定时器对象指针
 * @note 该函数通过定时器定期刷新图像条的显示内容
 */
void lv_imgbar_refresh_timer(lv_timer_t *timer);


/**
 * @brief 创建图像条对象
 * @param parent 父对象指针
 * @return 成功返回图像条对象指针，失败返回NULL
 * @note 该函数创建一个lv_imgbar对象并初始化
 */
lv_obj_t *lv_imgbar_create(lv_obj_t *parent);


/**
 * @brief 设置方向
 * @param img 图像条对象指针
 * @param dir 方向（lv_imgbar_dir_t）
 * @note 该函数设置图像条的显示方向
 */
void lv_imgbar_set_dir(lv_obj_t *img, lv_imgbar_dir_t dir);


/**
 * @brief 设置拖动标志
 * @param imgbar 图像条对象指针
 * @param en 拖动标志（true启用，false禁用）
 * @note 该函数设置是否启用拖动功能
 */
void lv_imgbar_set_drag(lv_obj_t *imgbar, bool en);


/**
 * @brief 设置值2（整数格式）
 * @param imgbar 图像条对象指针
 * @param value 值（整数格式）
 * @note 该函数设置图像条的进度值（整数格式）
 */
void lv_imgbar_set_value2(lv_obj_t *imgbar, int32_t value);


/**
 * @brief 设置值
 * @param img 图像条对象指针
 * @param value 值（Q24.8格式）
 * @note 该函数设置图像条的进度值（Q24.8格式）
 */
void lv_imgbar_set_value(lv_obj_t *img, int32_t value);


/**
 * @brief 设置模式
 * @param img 图像条对象指针
 * @param mode 模式（lv_imgbar_mode_t）
 * @note 该函数设置图像条的显示模式
 */
void lv_imgbar_set_mode(lv_obj_t *img, lv_imgbar_mode_t mode);


/**
 * @brief 设置用户回调函数
 * @param img 图像条对象指针
 * @param user_cb 用户回调函数指针
 * @note 该函数设置进度变化时的用户回调函数
 */
void lv_imgbar_set_user_cb(lv_obj_t *img, lv_imgbar_process_cb_t user_cb);


/**
 * @brief 设置指示器偏移
 * @param img 图像条对象指针
 * @param x X偏移
 * @param y Y偏移
 * @note 该函数设置指示器的偏移位置
 */
void lv_imgbar_set_indicator_offset(lv_obj_t *img, lv_coord_t x, lv_coord_t y);


/**
 * @brief 设置指示器X偏移
 * @param imgbar 图像条对象指针
 * @param x X偏移
 * @note 该函数设置指示器的X轴偏移位置
 */
void lv_imgbar_set_indicator_offset_x(lv_obj_t *imgbar, lv_coord_t x);


/**
 * @brief 设置指示器Y偏移
 * @param imgbar 图像条对象指针
 * @param y Y偏移
 * @note 该函数设置指示器的Y轴偏移位置
 */
void lv_imgbar_set_indicator_offset_y(lv_obj_t *imgbar, lv_coord_t y);


/**
 * @brief 设置前景图像
 * @param imgbar 图像条对象指针
 * @param img_fg 前景图像对象指针
 * @note 该函数设置图像条的前景图像
 */
void lv_imgbar_set_img_fg(lv_obj_t *imgbar, lv_obj_t *img_fg);


/**
 * @brief 设置指示器图像
 * @param imgbar 图像条对象指针
 * @param img_indicator 指示器图像对象指针
 * @note 该函数设置图像条的指示器图像
 */
void lv_imgbar_set_img_indicator(lv_obj_t *imgbar, lv_obj_t *img_indicator);


/**
 * @brief 设置背景图像
 * @param imgbar 图像条对象指针
 * @param img_bg 背景图像对象指针
 * @note 该函数设置图像条的背景图像
 */
void lv_imgbar_set_img_bg(lv_obj_t *imgbar, lv_obj_t *img_bg);


/**
 * @brief 获取背景图像
 * @param imgbar 图像条对象指针
 * @return 返回背景图像对象指针
 * @note 该函数获取图像条的背景图像
 */
lv_obj_t *lv_imgbar_get_img_bg(lv_obj_t *imgbar);


/**
 * @brief 获取指示器图像
 * @param imgbar 图像条对象指针
 * @return 返回指示器图像对象指针
 * @note 该函数获取图像条的指示器图像
 */
lv_obj_t *lv_imgbar_get_img_indicator(lv_obj_t *imgbar);


/**********************
 *      MACROS
 **********************/
#endif /*LVSF_USE_IMGBAR*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*_LVSF_IMGBAR_H_*/
