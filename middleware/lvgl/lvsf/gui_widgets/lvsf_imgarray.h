/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LVSF_IMG_ARRAY_H_
#define _LVSF_IMG_ARRAY_H_

#ifdef __cplusplus
extern "C" {
#endif


/*********************
 *      INCLUDES
 *********************/
#include "lvsf_conf_internal.h"
#include "lvsf_obj_ext.h"
#include "lvsf_baseimg.h"

#if LVSF_USE_IMGARRAY!=0


extern const lv_obj_class_t lv_imgarray_class;
/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**
 * @brief 图像数组控件数据结构
 * @note 该结构体用于管理一组图像的显示和数值映射
 */
typedef struct
{
    /*Ext. of ancestor*/
    lv_obj_t bg;
    lv_img_dsc_t **dsc_array;       /**<描述符数组*/
    lv_img_file_t file;             /**<文件信息*/

    uint16_t    index_start;        /**<起始索引*/
    uint16_t    index_end;          /**<结束索引*/

    uint16_t    index_negative;     /**<负号索引*/
    uint16_t    index_point;        /**<小数点索引*/
    uint16_t    index_unit;         /**<单位索引*/
    uint16_t    index_empty;        /**<空索引*/

    int16_t    interval;            /**<间隔*/
    lv_baseimg_type_t    img_type;  /**<图像类型*/
    lv_baseimg_src_type_t  src_type; /**<源类型*/

    bool leading_zero;              /**<前导零标志*/
    bool trailing_zero;             /**<尾随零标志*/
    uint8_t int_img_num;            /**<整数图像数量*/
    uint8_t float_img_num;          /**<小数图像数量*/
    lv_obj_t **int_img_tab;         /**<整数图像表*/
    lv_obj_t **float_img_tab;       /**<小数图像表*/
    lv_obj_t *unit_img;             /**<单位图像*/
    lv_obj_t *point_img;            /**<小数点图像*/
    lv_obj_t *negative_img;         /**<负号图像*/
    lv_obj_t *empty_img;            /**<空图像*/
} lv_imgarray_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * @brief 图像数组刷新定时器回调函数
 * @param timer 定时器对象指针
 * @note 该函数通过定时器定期刷新图像数组的显示内容
 */
void lv_imgarray_refresh_timer(lv_timer_t *timer);


/**
 * @brief 创建图像数组对象
 * @param parent 父对象指针
 * @return 成功返回图像数组对象指针，失败返回NULL
 * @note 该函数创建一个lv_imgarray对象并初始化
 */
lv_obj_t *lv_imgarray_create(lv_obj_t *parent);


/**
 * @brief 设置源数组
 * @param img 图像数组对象指针
 * @param dsc_array 描述符数组指针
 * @param index_start 起始索引
 * @param index_end 结束索引
 * @note 该函数设置图像数组的源数组
 */
void lv_imgarray_set_src_array(lv_obj_t *img, const lv_img_dsc_t **dsc_array,
                               int16_t index_start, int16_t index_end);


/**
 * @brief 设置源数组2
 * @param img 图像数组对象指针
 * @param file_path 文件路径
 * @param dsc_array 描述符数组指针
 * @param index_start 起始索引
 * @param index_end 结束索引
 * @note 该函数通过文件路径设置图像数组的源数组
 */
void lv_imgarray_set_src_array2(lv_obj_t *img, char *file_path, lv_img_file_data_t *dsc_array,
                                int16_t index_start, int16_t index_end);

/**
 * @brief 设置序列帧图片源和序号
 * @param img 图片数组对象指针
 * @param src 图片源
 * @param index_start 起始索引
 * @param index_end 结束索引
 */
void lv_imgarray_set_src_array3(lv_obj_t *img, const void *src, int16_t index_start, int16_t index_end);

/**
 * @brief 设置图像类型
 * @param img 图像数组对象指针
 * @param img_type 图像类型
 * @note 该函数设置图像数组的图像类型
 */
void lv_imgarray_set_img_type(lv_obj_t *img, lv_baseimg_type_t img_type);


/**
 * @brief 设置整数位数量
 * @param img 图像数组对象指针
 * @param num 整数位数量
 * @note 该函数设置图像数组的整数位图像数量
 */
void lv_imgarray_set_int_num(lv_obj_t *img, uint8_t num);


/**
 * @brief 设置小数位数量
 * @param img 图像数组对象指针
 * @param num 小数位数量
 * @note 该函数设置图像数组的小数位图像数量
 */
void lv_imgarray_set_float_num(lv_obj_t *img, uint8_t num);


/**
 * @brief 设置单位图像
 * @param img 图像数组对象指针
 * @param unit_img 单位图像对象指针
 * @note 该函数设置图像数组的单位显示图像
 */
void lv_imgarray_set_unit_img(lv_obj_t *img, lv_obj_t *unit_img);


/**
 * @brief 设置小数点索引
 * @param img 图像数组对象指针
 * @param idx 小数点索引
 * @note 该函数设置小数点对应的图像索引
 */
void lv_imgarray_set_point_idx(lv_obj_t *img, uint16_t idx);


/**
 * @brief 设置负号索引
 * @param img 图像数组对象指针
 * @param idx 负号索引
 * @note 该函数设置负号对应的图像索引
 */
void lv_imgarray_set_negative_idx(lv_obj_t *img, uint16_t idx);


/**
 * @brief 设置单位索引
 * @param img 图像数组对象指针
 * @param idx 单位索引
 * @note 该函数设置单位对应的图像索引
 */
void lv_imgarray_set_unit_idx(lv_obj_t *img, uint16_t idx);


/**
 * @brief 设置空索引
 * @param img 图像数组对象指针
 * @param idx 空索引
 * @note 该函数设置空位对应的图像索引
 */
void lv_imgarray_set_empty_idx(lv_obj_t *img, uint16_t idx);


/**
 * @brief 设置前导零标志
 * @param img 图像数组对象指针
 * @param leading_zero 前导零标志（true显示，false不显示）
 * @note 该函数设置是否显示前导零
 */
void lv_imgarray_set_leading_zero(lv_obj_t *img, bool leading_zero);


/**
 * @brief 设置尾随零标志
 * @param img 图像数组对象指针
 * @param trailing_zero 尾随零标志（true显示，false不显示）
 * @note 该函数设置是否显示尾随零
 */
void lv_imgarray_set_trailing_zero(lv_obj_t *img, bool trailing_zero);


/**
 * @brief 设置间隔
 * @param img 图像数组对象指针
 * @param interval 间隔值
 * @note 该函数设置图像之间的间隔
 */
void lv_imgarray_set_interval(lv_obj_t *img, int16_t interval);


/**
 * @brief 获取源数量
 * @param img 图像数组对象指针
 * @return 返回源数量
 * @note 该函数获取图像数组的源数量
 */
uint16_t lv_imgarray_get_src_num(lv_obj_t *img);


/**
 * @brief 设置值（Q24.8格式）
 * @param img 图像数组对象指针
 * @param value 值（Q24.8格式）
 * @note 该函数设置图像数组显示的数值（Q24.8格式）
 */
void lv_imgarray_set_value(lv_obj_t *img, int32_t value);


/**
 * @brief 设置值2（整数格式）
 * @param img 图像数组对象指针
 * @param value 值（整数格式）
 * @note 该函数设置图像数组显示的数值（整数格式）
 */
void lv_imgarray_set_value2(lv_obj_t *img, int32_t value);


/**********************
 *      MACROS
 **********************/
#endif /*LVSF_USE_IMG_ARRAY*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*_LVSF_IMG_ARRAY_H_*/
