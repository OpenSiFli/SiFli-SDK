/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*********************
 *      INCLUDES
 *********************/
#ifndef _LVSF_DATATYPE_H_
#define _LVSF_DATATYPE_H_
#include <rtthread.h>
#include "lvgl.h"

/**
 * @brief Q24.8格式转换为双精度浮点数
 * @param x Q24.8格式的整数
 * @return 转换后的双精度浮点数
 * @note Q24.8格式表示整数部分24位，小数部分8位
 */
#define Q24_Q_TO_DOUBLE(x) ((double)(x) / (1 << 8))

/**
 * @brief 双精度浮点数转换为Q24.8格式
 * @param x 双精度浮点数
 * @return Q24.8格式的整数
 * @note Q24.8格式表示整数部分24位，小数部分8位
 */
#define Q24_DOUBLE_TO_Q(x) ((int32_t)((x) * (1 << 8)))

/**
 * @brief 异常状态枚举
 * @note 定义了数据库数据的各种异常状态
 */
enum
{
    SFAT_ABNORMAL_STATUS_OK = 0,          //系统未获取数据
    SFAT_ABNORMAL_STATUS_NULL = 0x10000000,          //系统未获取数据
    SFAT_ABNORMAL_STATUS_UNIT_MISMATCH = 0x20000000, //当前单位与预设单位不匹配
    SFAT_ABNORMAL_STATUS_OUT_RANGE = 0x40000000,     //当前数据超出预设范围
    SFAT_ABNORMAL_STATUS_INVALID = 0x80000000,       //当前数据无效
};
typedef int32_t database_abnormal_status_t;

/**
 * @brief 数据库类型枚举
 * @note 定义了支持的各种数据类型
 */
enum
{
    DATABASE_TYPE_NULL,
    DATABASE_TYPE_BOOL,
    DATABASE_TYPE_INT8,
    DATABASE_TYPE_INT16,
    DATABASE_TYPE_INT32,
    DATABASE_TYPE_UINT8,
    DATABASE_TYPE_UINT16,
    DATABASE_TYPE_UINT32,
    DATABASE_TYPE_INT8_ARRAY,
    DATABASE_TYPE_INT16_ARRAY,
    DATABASE_TYPE_INT32_ARRAY,
    DATABASE_TYPE_UINT8_ARRAY,
    DATABASE_TYPE_UINT16_ARRAY,
    DATABASE_TYPE_UINT32_ARRAY,
    DATABASE_TYPE_FLOAT,
    DATABASE_TYPE_FLOAT_ARRAY,
    DATABASE_TYPE_DOUBLE,
    DATABASE_TYPE_DOUBLE_ARRAY,
    DATABASE_TYPE_STRING,
    DATABASE_TYPE_Q24_8,
};
typedef uint8_t database_type_t;

/**
 * @brief 数据库分析类型枚举
 * @note 定义了数据的分析和显示类型
 */
enum
{
    DATABASE_TYPE_NORMAL,
    DATABASE_TYPE_PERCENT,
    DATABASE_TYPE_TIMESTAMP,
    DATABASE_TYPE_DATE_H_M,         //小时和分钟
    DATABASE_TYPE_DATE_M_S,         //分钟和秒
    DATABASE_TYPE_DATE_H_M_S,       //小时、分钟和秒
    DATABASE_TYPE_DATE_M_D,         //月和日
    DATABASE_TYPE_DATE_W_D,         //周和日
};
typedef uint8_t database_analysis_type_t;


/**
 * @brief 控件数据刷新回调函数类型
 * @param idx 控件索引
 * @note 当控件需要刷新数据时调用该回调函数
 */
typedef void (*lv_widgets_data_refresh_cb)(uint16_t idx);


/**
 * @brief 数据类型结构体
 */
typedef struct
{
    lv_widgets_data_refresh_cb data_refresh_cb;    /**<数据刷新回调函数*/

} lv_datatype_t;


/**
 * @brief 获取数据刷新回调函数
 * @return 返回数据刷新回调函数指针
 * @note 该函数获取全局的数据刷新回调函数
 */
lv_widgets_data_refresh_cb lv_datatype_get_refresh_cb(void);


/**
 * @brief 设置数据刷新回调函数
 * @param cb 数据刷新回调函数指针
 * @note 该函数设置全局的数据刷新回调函数
 */
void lv_datatype_set_refresh_cb(lv_widgets_data_refresh_cb cb);



#endif  /*_LVSF_DATATYPE_H_*/
