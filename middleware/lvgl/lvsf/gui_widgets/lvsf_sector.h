/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*********************
 *      INCLUDES
 *********************/
#ifndef _LVSF_SECTOR_H
#define _LVSF_SECTOR_H

#ifdef __cplusplus
extern "C" {
#endif



/*********************
 *      INCLUDES
 *********************/

#include "lvsf_conf_internal.h"
#include "lvsf_obj_ext.h"

#if LVSF_USE_SECTOR != 0


extern const lv_obj_class_t lv_sector_class;

/*Testing of dependencies*/
//#if LV_USE_OBJMASK == 0
//#error "lvsf_sector: lv_bar is required. Enable it in lv_conf.h (LV_USE_OBJMASK  1) "
//#endif


/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**
 * @brief 扇形控件数据结构
 * @note 该结构体用于管理扇形区域的显示和交互
 */
typedef struct
{
    lv_img_t bg;                    /**<背景图像*/
    int16_t angle_start;            /**<起始角度*/
    int16_t angle_end;              /**<结束角度*/

    float angle_rest;        //拖动时的变化角度,由于设置的角度是整数，所以有可能出现角度的浮点误差
    /**<拖动时的变化角度,由于设置的角度是整数，所以有可能出现角度的浮点误差*/

    uint32_t drag : 1;          /**<允许拖动修改进度*/
    uint32_t stop_timer : 1;


    uint8_t *data;              /**<申请的mask数据*/

    lv_obj_t *img_indicator;    /**<指示器图像*/
    lv_obj_t *img_bg;           /**<背景图像*/

    lv_coord_t offset_x;        /**<X偏移*/
    lv_coord_t offset_y;        /**<Y偏移*/

    lv_draw_mask_map_param_t m; /**<掩码参数*/
    int16_t mask_id;            /**<掩码ID*/
} lv_sector_t;


/**
 * @brief 扇形刷新定时器回调函数
 * @param timer 定时器对象指针
 * @note 该函数通过定时器定期刷新扇形的显示内容
 */
void lv_sector_refresh_timer(lv_timer_t *timer);


/**
 * @brief 创建扇形对象
 * @param parent 父对象指针
 * @return 成功返回扇形对象指针，失败返回NULL
 * @note 该函数创建一个lv_sector对象并初始化
 */
lv_obj_t *lv_sector_create(lv_obj_t *parent);


/**
 * @brief 刷新掩码范围
 * @param sector 扇形对象指针
 * @param min 最小值
 * @param max 最大值
 * @param value 当前值
 * @note 该函数刷新扇形的掩码范围
 */
void lv_sector_refresh_mask_range(lv_obj_t *sector, int32_t min, int32_t max, uint8_t value);


/**
 * @brief 验证扇形
 * @param sector 扇形对象指针
 * @note 该函数验证扇形的配置
 */
void lv_sector_validate(lv_obj_t *sector);


/**
 * @brief 设置值
 * @param sector 扇形对象指针
 * @param value 值
 * @note 该函数设置扇形的值
 */
void lv_sector_set_value(lv_obj_t *sector, int32_t value);


/**
 * @brief 设置拖动标志
 * @param sector 扇形对象指针
 * @param en 拖动标志（true启用，false禁用）
 * @note 该函数设置是否启用拖动功能
 */
void lv_sector_set_drag(lv_obj_t *sector, bool en);


/**
 * @brief 设置指示器图像
 * @param sector 扇形对象指针
 * @param img_indicator 指示器图像对象指针
 * @note 该函数设置扇形的指示器图像
 */
void lv_sector_set_img_indicator(lv_obj_t *sector, lv_obj_t *img_indicator);


/**
 * @brief 设置指示器偏移
 * @param sector 扇形对象指针
 * @param x X偏移
 * @param y Y偏移
 * @note 该函数设置扇形指示器的偏移位置
 */
void lv_sector_set_indicator_offset(lv_obj_t *sector, lv_coord_t x, lv_coord_t y);


/**
 * @brief 设置指示器X偏移
 * @param sector 扇形对象指针
 * @param x X偏移
 * @note 该函数设置扇形指示器的X轴偏移位置
 */
void lv_sector_set_indicator_offset_x(lv_obj_t *sector, lv_coord_t x);


/**
 * @brief 设置指示器Y偏移
 * @param sector 扇形对象指针
 * @param y Y偏移
 * @note 该函数设置扇形指示器的Y轴偏移位置
 */
void lv_sector_set_indicator_offset_y(lv_obj_t *sector, lv_coord_t y);


/**
 * @brief 获取起始角度
 * @param sector 扇形对象指针
 * @return 返回起始角度
 * @note 该函数获取扇形的起始角度
 */
int16_t lv_sector_get_start_angle(lv_obj_t *sector);


/**
 * @brief 获取结束角度
 * @param sector 扇形对象指针
 * @return 返回结束角度
 * @note 该函数获取扇形的结束角度
 */
int16_t lv_sector_get_end_angle(lv_obj_t *sector);


/**
 * @brief 获取背景图像
 * @param sector 扇形对象指针
 * @return 返回背景图像对象指针
 * @note 该函数获取扇形的背景图像
 */
lv_obj_t *lv_sector_get_img_bg(lv_obj_t *sector);


/**
 * @brief 获取指示器图像
 * @param sector 扇形对象指针
 * @return 返回指示器图像对象指针
 * @note 该函数获取扇形的指示器图像
 */
lv_obj_t *lv_sector_get_img_indicator(lv_obj_t *sector);


/**********************
 *      MACROS
 **********************/

#endif /*LVSF_USE_SECTOR*/

#ifdef __cplusplus
} /* extern "C" */
#endif


#endif /* _LVSF_SECTOR_H */
