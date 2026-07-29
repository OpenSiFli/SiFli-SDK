/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*********************
 *      INCLUDES
 *********************/
#ifndef _LVSF_OBJ_EXT_H
#define _LVSF_OBJ_EXT_H


#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"
#include "lvsf_conf_internal.h"

#if LVSF_USE_OBJ_EXT

/*********************
 *      INCLUDES
 *********************/

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**
 * @brief 获取多数据回调函数类型
 * @param obj 对象指针
 * @param id_tab ID表指针
 * @param id_num ID数量
 * @return 返回数据值
 * @note 通过该回调函数获取多个数据
 */
typedef int32_t(*lv_obj_gmdata_cb)(struct _lv_obj_t *obj, uint32_t *id_tab, uint8_t id_num);

/**
 * @brief 获取数据回调函数类型
 * @param obj 对象指针
 * @param id 数据ID
 * @return 返回数据值
 * @note 通过该回调函数获取单个数据
 */
typedef int32_t(*lv_obj_gdata_cb)(struct _lv_obj_t *obj, uint32_t id);

/**
 * @brief 设置数据回调函数类型
 * @param id 数据ID
 * @param data 数据值
 * @return 返回操作结果
 * @note 通过该回调函数设置数据
 */
typedef int32_t(*lv_obj_sdata_cb)(uint32_t id, int32_t data);

/**
 * @brief 基础对象扩展属性结构体
 * @note 该结构体扩展了LVGL对象的属性，用于数据绑定和控件管理
 */
typedef struct
{
    uint8_t     ori_align;      /**<向原始坐标的对齐模式*/
    uint8_t     data_type;      /**<用来记录cb传入的数据类型*/
    uint8_t     id_num;         /**<用来记录id_tab的长度*/
    lv_coord_t  ori_x;          /**<原始的X坐标*/
    lv_coord_t  ori_y;          /**<原始的Y坐标*/
    lv_coord_t  ori_w;          /**<原始的宽度*/
    lv_coord_t  ori_h;          /**<原始的高度*/

    uint32_t     hidden_rule;    /**<Under what circumstances can it be hidden, database_abnormal_status_t*/    /**<控件隐藏规则*/
    int32_t     value_min;      /**<Range of bound data changes*/  /**<绑定数据的最小值， 整型数据，不是Q24.8*/
    int32_t     value_max;      /**<Range of bound data changes*/  /**<绑定数据的最大值， 整型数据，不是Q24.8*/
    int32_t     scale_min;      /**<scale, e.g: percent for label; angle for pointer*/    /**<控件变化范围的最小值， 整型数据，不是Q24.8*/
    int32_t     scale_max;      /**<scale, e.g: percent for label; angle for pointer*/    /**<控件变化范围的最大值， 整型数据，不是Q24.8*/

    uint32_t    source_unit_id; /**<绑定的数据源的原始单位id*/
    uint32_t    target_unit_id; /**<绑定的数据源的目标单位id*/
    uint32_t *id_tab;        /**<表盘工具绑定的数据id*/
    void *time;          /**<record time*/
    lv_timer_t *timer;            /**<刷新定时器*/
    lv_obj_gmdata_cb gmdata_cb;       /**<get mult data by this callback, used by label*/ /**<通过此回调获取多数据，用于标签*/
    lv_obj_gdata_cb gdata_cb;       /**<get data by this callback*/ /**<通过此回调获取数据*/
    lv_obj_sdata_cb sdata_cb;       /**<set data by this callback*/ /**<通过此回调设置数据*/
} lv_obj_ext_t;


/**
 * @brief 将事件代码转换为名称
 * @param code 事件代码
 * @return 返回事件名称字符串
 * @note 该函数将LVGL事件代码转换为可读的字符串
 */
char *lv_event_code_to_name(lv_event_code_t code);


/**
 * @brief 删除对象扩展属性1
 * @param obj 对象指针
 * @note 该函数删除对象的扩展属性
 */
void lv_obj_del_ext_attr1(lv_obj_t *obj);


/**
 * @brief 获取对象扩展属性1
 * @param obj 对象指针
 * @return 返回扩展属性指针
 * @note 该函数获取对象的扩展属性
 */
lv_obj_ext_t *lv_obj_get_ext_attr1(lv_obj_t *obj);


/**
 * @brief 获取对象用户数据
 * @param obj 对象指针
 * @return 返回用户数据指针
 * @note 该函数获取对象的用户自定义数据
 */
void *lv_obj_get_tool_user_data(lv_obj_t *obj);


/**
 * @brief 设置对象用户数据
 * @param obj 对象指针
 * @param user_data 用户数据指针
 * @note 该函数设置对象的用户自定义数据
 */
void lv_obj_set_tool_user_data(lv_obj_t *obj, void *user_data);


/**
 * @brief 对齐到原始位置
 * @param obj 对象指针
 * @note 该函数将对象对齐到其原始位置
 */
void lv_obj_align_to_ori_pos(lv_obj_t *obj);


/**
 * @brief 创建刷新定时器
 * @param obj 对象指针
 * @param period 周期（毫秒）
 * @param timer_cb 定时器回调函数
 * @note 该函数为对象创建一个刷新定时器
 */
void lv_obj_create_refresh_timer(lv_obj_t *obj, uint32_t period, lv_timer_cb_t timer_cb);


/**
 * @brief 开始刷新
 * @param obj 对象指针
 * @note 该函数开始对象的刷新
 */
void lv_obj_refresh_start(lv_obj_t *obj);


/**
 * @brief 停止刷新
 * @param obj 对象指针
 * @note 该函数停止对象的刷新
 */
void lv_obj_refresh_stop(lv_obj_t *obj);


/**
 * @brief 设置原始对齐模式
 * @param obj 对象指针
 * @param ori_align 原始对齐模式
 * @note 该函数设置对象的原始对齐模式
 */
void lv_obj_set_ori_align(lv_obj_t *obj, uint8_t ori_align);


/**
 * @brief 设置数据类型
 * @param obj 对象指针
 * @param data_type 数据类型
 * @note 该函数设置对象的数据类型
 */
void lv_obj_set_data_type(lv_obj_t *obj, uint8_t data_type);


/**
 * @brief 设置原始宽度
 * @param obj 对象指针
 * @param w 宽度
 * @note 该函数设置对象的原始宽度
 */
void lv_obj_set_ori_size_w(lv_obj_t *obj, lv_coord_t w);


/**
 * @brief 设置原始高度
 * @param obj 对象指针
 * @param h 高度
 * @note 该函数设置对象的原始高度
 */
void lv_obj_set_ori_size_h(lv_obj_t *obj, lv_coord_t h);


/**
 * @brief 设置原始大小
 * @param obj 对象指针
 * @param w 宽度
 * @param h 高度
 * @note 该函数设置对象的原始大小
 */
void lv_obj_set_ori_size(lv_obj_t *obj, lv_coord_t w, lv_coord_t h);


/**
 * @brief 设置原始X坐标
 * @param obj 对象指针
 * @param x X坐标
 * @note 该函数设置对象的原始X坐标
 */
void lv_obj_set_ori_pos_x(lv_obj_t *obj, lv_coord_t x);


/**
 * @brief 设置原始Y坐标
 * @param obj 对象指针
 * @param y Y坐标
 * @note 该函数设置对象的原始Y坐标
 */
void lv_obj_set_ori_pos_y(lv_obj_t *obj, lv_coord_t y);


/**
 * @brief 设置原始位置
 * @param obj 对象指针
 * @param x X坐标
 * @param y Y坐标
 * @note 该函数设置对象的原始位置
 */
void lv_obj_set_ori_pos(lv_obj_t *obj, lv_coord_t x, lv_coord_t y);


/**
 * @brief 设置隐藏规则
 * @param obj 对象指针
 * @param hidden_rule 隐藏规则
 * @note 该函数设置对象的隐藏规则
 */
void lv_obj_set_hidden_rule(lv_obj_t *obj, uint32_t hidden_rule);


/**
 * @brief 设置范围最小值
 * @param obj 对象指针
 * @param min 最小值
 * @note 该函数设置对象的范围最小值
 */
void lv_obj_set_range_value_min(lv_obj_t *obj, int32_t min);


/**
 * @brief 设置范围最大值
 * @param obj 对象指针
 * @param max 最大值
 * @note 该函数设置对象的范围最大值
 */
void lv_obj_set_range_value_max(lv_obj_t *obj, int32_t max);


/**
 * @brief 设置范围
 * @param obj 对象指针
 * @param min 最小值
 * @param max 最大值
 * @note 该函数设置对象的范围
 */
void lv_obj_set_range_value(lv_obj_t *obj, int32_t min, int32_t max);


/**
 * @brief 设置缩放最小值
 * @param obj 对象指针
 * @param min 最小值
 * @note 该函数设置对象的缩放最小值
 */
void lv_obj_set_range_scale_min(lv_obj_t *obj, int32_t min);


/**
 * @brief 设置缩放最大值
 * @param obj 对象指针
 * @param max 最大值
 * @note 该函数设置对象的缩放最大值
 */
void lv_obj_set_range_scale_max(lv_obj_t *obj, int32_t max);


/**
 * @brief 设置缩放范围
 * @param obj 对象指针
 * @param min 最小值
 * @param max 最大值
 * @note 该函数设置对象的缩放范围
 */
void lv_obj_set_range_scale(lv_obj_t *obj, int32_t min, int32_t max);


/**
 * @brief 设置单位ID
 * @param obj 对象指针
 * @param source_unit_id 源单位ID
 * @param target_unit_id 目标单位ID
 * @note 该函数设置对象的单位ID
 */
void lv_obj_set_unit_id(lv_obj_t *obj, uint32_t source_unit_id, uint32_t target_unit_id);


/**
 * @brief 设置源ID
 * @param obj 对象指针
 * @param source_id 源ID数组
 * @param num ID数量
 * @note 该函数设置对象的源ID
 */
void lv_obj_set_source_id(lv_obj_t *obj, uint32_t *source_id, uint8_t num);


/**
 * @brief 设置获取多数据回调函数
 * @param obj 对象指针
 * @param cb 回调函数指针
 * @note 该函数设置获取多数据的回调函数
 */
void lv_obj_set_gmdata_cb(lv_obj_t *obj, lv_obj_gmdata_cb cb);


/**
 * @brief 设置获取数据回调函数
 * @param obj 对象指针
 * @param cb 回调函数指针
 * @note 该函数设置获取数据的回调函数
 */
void lv_obj_set_gdata_cb(lv_obj_t *obj, lv_obj_gdata_cb cb);


/**
 * @brief 设置设置数据回调函数
 * @param obj 对象指针
 * @param cb 回调函数指针
 * @note 该函数设置设置数据的回调函数
 */
void lv_obj_set_sdata_cb(lv_obj_t *obj, lv_obj_sdata_cb cb);


/**
 * @brief 获取原始对齐模式
 * @param obj 对象指针
 * @return 返回原始对齐模式
 * @note 该函数获取对象的原始对齐模式
 */
int32_t lv_obj_get_ori_align(lv_obj_t *obj);


/**
 * @brief 获取数据类型
 * @param obj 对象指针
 * @return 返回数据类型
 * @note 该函数获取对象的数据类型
 */
uint8_t lv_obj_get_data_type(lv_obj_t *obj);


/**
 * @brief 获取原始宽度
 * @param obj 对象指针
 * @return 返回原始宽度
 * @note 该函数获取对象的原始宽度
 */
lv_coord_t lv_obj_get_ori_size_w(lv_obj_t *obj);


/**
 * @brief 获取原始高度
 * @param obj 对象指针
 * @return 返回原始高度
 * @note 该函数获取对象的原始高度
 */
lv_coord_t lv_obj_get_ori_size_h(lv_obj_t *obj);


/**
 * @brief 获取原始X坐标
 * @param obj 对象指针
 * @return 返回原始X坐标
 * @note 该函数获取对象的原始X坐标
 */
lv_coord_t lv_obj_get_ori_pos_x(lv_obj_t *obj);


/**
 * @brief 获取原始Y坐标
 * @param obj 对象指针
 * @return 返回原始Y坐标
 * @note 该函数获取对象的原始Y坐标
 */
lv_coord_t lv_obj_get_ori_pos_y(lv_obj_t *obj);


/**
 * @brief 获取隐藏规则
 * @param obj 对象指针
 * @return 返回隐藏规则
 * @note 该函数获取对象的隐藏规则
 */
int32_t lv_obj_get_hidden_rule(lv_obj_t *obj);


/**
 * @brief 获取范围最小值
 * @param obj 对象指针
 * @return 返回范围最小值
 * @note 该函数获取对象的范围最小值
 */
int32_t lv_obj_get_range_value_min(lv_obj_t *obj);


/**
 * @brief 获取范围最大值
 * @param obj 对象指针
 * @return 返回范围最大值
 * @note 该函数获取对象的范围最大值
 */
int32_t lv_obj_get_range_value_max(lv_obj_t *obj);


/**
 * @brief 获取缩放最小值
 * @param obj 对象指针
 * @return 返回缩放最小值
 * @note 该函数获取对象的缩放最小值
 */
int32_t lv_obj_get_range_scale_min(lv_obj_t *obj);


/**
 * @brief 获取缩放最大值
 * @param obj 对象指针
 * @return 返回缩放最大值
 * @note 该函数获取对象的缩放最大值
 */
int32_t lv_obj_get_range_scale_max(lv_obj_t *obj);


/**
 * @brief 获取源ID
 * @param obj 对象指针
 * @param index 索引
 * @return 返回源ID
 * @note 该函数获取对象指定索引的源ID
 */
uint32_t lv_obj_get_source_id(lv_obj_t *obj, uint8_t index);


/**
 * @brief 获取刷新定时器
 * @param obj 对象指针
 * @return 返回刷新定时器指针
 * @note 该函数获取对象的刷新定时器
 */
lv_timer_t *lv_obj_get_refresh_timer(lv_obj_t *obj);


/**
 * @brief 隐藏规则检查
 * @param obj 对象指针
 * @param ret 返回值
 * @return 返回检查结果
 * @note 该函数检查对象的隐藏规则
 */
bool lv_obj_hidden_rule_check(lv_obj_t *obj, int32_t ret);

/**********************
 *      MACROS
 **********************/

#endif /*LVSF_USE_OBJ_EXT*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
