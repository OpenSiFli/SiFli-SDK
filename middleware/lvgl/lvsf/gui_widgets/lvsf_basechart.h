/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*********************
 *      INCLUDES
 *********************/
#ifndef _LV_BASECHART_H
#define _LV_BASECHART_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#if LVSF_USE_BASECHART != 0

/*Testing of dependencies*/
#if LV_USE_CHART== 0
#error "lvsf_basechart: lv_chart is required. Enable it in lv_conf.h (LV_USE_CONT  1) "
#endif

extern const lv_obj_class_t lv_basechart_class;


/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**
 * @brief 基础图表颜色分区结构体
 */
typedef struct
{
    int32_t value;    /**<颜色分区值*/
    uint32_t color;   /**<颜色值*/
} lv_basechart_color_t;

/**
 * @brief 柱状图类型枚举
 */
enum
{
    LV_BASECHART_COLUMN_TYPE_COMM,      //正常的柱状图
    LV_BASECHART_COLUMN_TYPE_DOUBLE,    //有两个端点的柱状图，每个柱子需要两个数据
    LV_BASECHART_COLUMN_TYPE_MULT,      //柱状图上的散点，如果有相连数据的点，连成一个柱子
};
typedef uint8_t lv_basechart_column_type_t;


/**
 * @brief 基础图表数据结构
 */
typedef struct
{
    lv_chart_t bg;

    uint32_t color_num : 3;                  //柱状图颜色分区数量
    uint32_t series_num : 3;                //数据源的数量
    uint32_t column_type : 2;               //柱状图类型：lv_basechart_column_type_t

    uint8_t *simple_multiple;           //每条数据源的数据取值间隔, 如果是LV_BASECHART_COLUMN_TYPE_MULT类型，每个柱子最大的数据数量
    lv_basechart_color_t *color_tab;    //柱状图颜色分区
    uint32_t *chart_data_addr;           //原始图表数据源的数组

} lv_basechart_t;


/**
 * @brief 基础图表刷新定时器回调函数
 * @param timer 定时器对象指针
 * @note 该函数通过定时器定期调用用户注册的回调函数来更新图表内容
 */
void lv_basechart_refresh_timer(lv_timer_t *timer);


/**
 * @brief 创建基础图表对象
 * @param parent 父对象指针
 * @return 成功返回图表对象指针，失败返回NULL
 * @note 该函数创建一个lv_basechart对象并初始化
 */
lv_obj_t *lv_basechart_create(lv_obj_t *parent);


/**
 * @brief 设置柱状图类型
 * @param chart 图表对象指针
 * @param column_type 柱状图类型（lv_basechart_column_type_t）
 * @param column_mult_num 每个柱子的最大数据数量（用于LV_BASECHART_COLUMN_TYPE_MULT类型）
 * @note 该函数设置柱状图的显示类型
 */
void lv_basechart_set_column_type(lv_obj_t *chart, lv_basechart_column_type_t column_type, uint8_t column_mult_num);


/**
 * @brief 设置颜色分区
 * @param chart 图表对象指针
 * @param num 颜色分区数量
 * @param color 颜色分区数组指针
 * @note 该函数设置图表的颜色分区，不同值范围显示不同颜色
 */
void lv_basechart_set_color(lv_obj_t *chart, uint8_t num, lv_basechart_color_t *color);


/**
 * @brief 添加简单倍数
 * @param chart 图表对象指针
 * @param simple_multiple 简单倍数数组指针
 * @note 该函数设置每条数据源的数据取值间隔
 */
void lv_basechart_add_simple_multiple(lv_obj_t *chart, uint8_t simple_multiple);


/* from lv_chart.h */

/**
 * @brief 设置图表类型
 * @param obj 图表对象指针
 * @param type 图表类型
 * @note 该函数设置图表的显示类型（折线图、柱状图等）
 */
void lv_basechart_set_type(lv_obj_t *obj, lv_chart_type_t type);


/**
 * @brief 设置点数量
 * @param obj 图表对象指针
 * @param cnt 点的数量
 * @note 该函数设置图表中每个数据系列的点数量
 */
void lv_basechart_set_point_count(lv_obj_t *obj, uint16_t cnt);


/**
 * @brief 设置范围
 * @param obj 图表对象指针
 * @param axis 坐标轴（X轴或Y轴）
 * @param min 最小值
 * @param max 最大值
 * @note 该函数设置图表的坐标轴范围
 */
void lv_basechart_set_range(lv_obj_t *obj, lv_chart_axis_t axis, lv_coord_t min, lv_coord_t max);


/**
 * @brief 设置更新模式
 * @param obj 图表对象指针
 * @param update_mode 更新模式
 * @note 该函数设置图表的更新模式
 */
void lv_basechart_set_update_mode(lv_obj_t *obj, lv_chart_update_mode_t update_mode);


/**
 * @brief 设置分割线数量
 * @param obj 图表对象指针
 * @param hdiv 水平分割线数量
 * @param vdiv 垂直分割线数量
 * @note 该函数设置图表的网格线数量
 */
void lv_basechart_set_div_line_count(lv_obj_t *obj, uint8_t hdiv, uint8_t vdiv);


/**
 * @brief 设置X轴缩放
 * @param obj 图表对象指针
 * @param zoom_x X轴缩放比例
 * @note 该函数设置图表X轴的缩放比例
 */
void lv_basechart_set_zoom_x(lv_obj_t *obj, uint16_t zoom_x);


/**
 * @brief 设置Y轴缩放
 * @param obj 图表对象指针
 * @param zoom_y Y轴缩放比例
 * @note 该函数设置图表Y轴的缩放比例
 */
void lv_basechart_set_zoom_y(lv_obj_t *obj, uint16_t zoom_y);


/**
 * @brief 获取X轴缩放
 * @param obj 图表对象指针
 * @return 返回X轴缩放比例
 */
uint16_t lv_basechart_get_zoom_x(const lv_obj_t *obj);


/**
 * @brief 获取Y轴缩放
 * @param obj 图表对象指针
 * @return 返回Y轴缩放比例
 */
uint16_t lv_basechart_get_zoom_y(const lv_obj_t *obj);


/**
 * @brief 设置轴刻度
 * @param obj 图表对象指针
 * @param axis 坐标轴
 * @param major_len 主刻度长度
 * @param minor_len 次刻度长度
 * @param major_cnt 主刻度数量
 * @param minor_cnt 次刻度数量
 * @param label_en 是否显示标签
 * @param draw_size 绘制大小
 * @note 该函数设置图表轴的刻度参数
 */
void lv_basechart_set_axis_tick(lv_obj_t *obj, lv_chart_axis_t axis, lv_coord_t major_len, lv_coord_t minor_len, lv_coord_t major_cnt, lv_coord_t minor_cnt, bool label_en, lv_coord_t draw_size);


/**
 * @brief 获取图表类型
 * @param obj 图表对象指针
 * @return 返回图表类型
 */
lv_chart_type_t lv_basechart_get_type(const lv_obj_t *obj);


/**
 * @brief 获取点数量
 * @param obj 图表对象指针
 * @return 返回点数量
 */
uint16_t lv_basechart_get_point_count(const lv_obj_t *obj);


/**
 * @brief 获取X轴起始点
 * @param obj 图表对象指针
 * @param ser 数据系列
 * @return 返回X轴起始点索引
 */
uint16_t lv_basechart_get_x_start_point(const lv_obj_t *obj, lv_chart_series_t *ser);


/**
 * @brief 获取指定ID的点位置
 * @param obj 图表对象指针
 * @param ser 数据系列
 * @param id 点ID
 * @param p_out 用于返回位置坐标的指针
 * @note 该函数获取指定点在图表中的坐标位置
 */
void lv_basechart_get_point_pos_by_id(lv_obj_t *obj, lv_chart_series_t *ser, uint16_t id, lv_point_t *p_out);


/**
 * @brief 刷新图表
 * @param obj 图表对象指针
 * @note 该函数强制刷新图表显示
 */
void lv_basechart_refresh(lv_obj_t *obj);


/**
 * @brief 添加数据系列
 * @param obj 图表对象指针
 * @param color 数据系列颜色
 * @param axis 坐标轴
 * @return 返回数据系列指针
 * @note 该函数添加一个新的数据系列到图表
 */
lv_chart_series_t *lv_basechart_add_series(lv_obj_t *obj, lv_color_t color, lv_chart_axis_t axis);


/**
 * @brief 删除数据系列
 * @param obj 图表对象指针
 * @param series 要删除的数据系列
 * @note 该函数从图表中删除指定的数据系列
 */
void lv_basechart_remove_series(lv_obj_t *obj, lv_chart_series_t *series);


/**
 * @brief 隐藏数据系列
 * @param chart 图表对象指针
 * @param series 数据系列
 * @param hide 隐藏标志（true隐藏，false显示）
 * @note 该函数设置数据系列的隐藏状态
 */
void lv_basechart_hide_series(lv_obj_t *chart, lv_chart_series_t *series, bool hide);


/**
 * @brief 设置数据系列颜色
 * @param chart 图表对象指针
 * @param series 数据系列
 * @param color 颜色值
 * @note 该函数设置数据系列的颜色
 */
void lv_basechart_set_series_color(lv_obj_t *chart, lv_chart_series_t *series, lv_color_t color);


/**
 * @brief 设置X轴起始点
 * @param obj 图表对象指针
 * @param ser 数据系列
 * @param id 起始点ID
 * @note 该函数设置数据系列的X轴起始点
 */
void lv_basechart_set_x_start_point(lv_obj_t *obj, lv_chart_series_t *ser, uint16_t id);


/**
 * @brief 获取下一个数据系列
 * @param chart 图表对象指针
 * @param ser 当前数据系列
 * @return 返回下一个数据系列指针
 * @note 该函数获取图表中的下一个数据系列
 */
lv_chart_series_t *lv_basechart_get_series_next(const lv_obj_t *chart, const lv_chart_series_t *ser);


/**
 * @brief 添加光标
 * @param obj 图表对象指针
 * @param color 光标颜色
 * @param dir 光标方向
 * @return 返回光标指针
 * @note 该函数添加一个光标到图表
 */
lv_chart_cursor_t *lv_basechart_add_cursor(lv_obj_t *obj, lv_color_t color, lv_dir_t dir);


/**
 * @brief 设置光标位置
 * @param chart 图表对象指针
 * @param cursor 光标
 * @param pos 位置坐标
 * @note 该函数设置光标在图表中的位置
 */
void lv_basechart_set_cursor_pos(lv_obj_t *chart, lv_chart_cursor_t *cursor, lv_point_t *pos);


/**
 * @brief 设置光标点
 * @param chart 图表对象指针
 * @param cursor 光标
 * @param ser 数据系列
 * @param point_id 点ID
 * @note 该函数设置光标指向的数据点
 */
void lv_basechart_set_cursor_point(lv_obj_t *chart, lv_chart_cursor_t *cursor, lv_chart_series_t *ser, uint16_t point_id);


/**
 * @brief 获取光标点
 * @param chart 图表对象指针
 * @param cursor 光标
 * @return 返回光标指向的点坐标
 * @note 该函数获取光标当前指向的数据点坐标
 */
lv_point_t lv_basechart_get_cursor_point(lv_obj_t *chart, lv_chart_cursor_t *cursor);


/**
 * @brief 设置所有值
 * @param obj 图表对象指针
 * @param ser 数据系列
 * @param value 值
 * @note 该函数设置数据系列的所有点为同一个值
 */
void lv_basechart_set_all_value(lv_obj_t *obj, lv_chart_series_t *ser, lv_coord_t value);


/**
 * @brief 设置下一个值
 * @param obj 图表对象指针
 * @param ser 数据系列
 * @param value 值
 * @note 该函数设置数据系列的下一个点的值
 */
void lv_basechart_set_next_value(lv_obj_t *obj, lv_chart_series_t *ser, lv_coord_t value);


/**
 * @brief 设置下一个值（X/Y）
 * @param obj 图表对象指针
 * @param ser 数据系列
 * @param x_value X值
 * @param y_value Y值
 * @note 该函数设置数据系列的下一个点的X和Y值
 */
void lv_basechart_set_next_value2(lv_obj_t *obj, lv_chart_series_t *ser, lv_coord_t x_value, lv_coord_t y_value);


/**
 * @brief 设置指定ID的值
 * @param obj 图表对象指针
 * @param ser 数据系列
 * @param id 点ID
 * @param value 值
 * @note 该函数设置数据系列指定ID的点的值
 */
void lv_basechart_set_value_by_id(lv_obj_t *obj, lv_chart_series_t *ser, uint16_t id, lv_coord_t value);


/**
 * @brief 设置指定ID的值（X/Y）
 * @param obj 图表对象指针
 * @param ser 数据系列
 * @param id 点ID
 * @param x_value X值
 * @param y_value Y值
 * @note 该函数设置数据系列指定ID的点的X和Y值
 */
void lv_basechart_set_value_by_id2(lv_obj_t *obj, lv_chart_series_t *ser, uint16_t id, lv_coord_t x_value, lv_coord_t y_value);


/**
 * @brief 设置外部Y数组
 * @param obj 图表对象指针
 * @param ser 数据系列
 * @param array Y数组
 * @note 该函数设置数据系列的外部Y轴数组
 */
void lv_basechart_set_ext_y_array(lv_obj_t *obj, lv_chart_series_t *ser, lv_coord_t array[]);


/**
 * @brief 设置外部X数组
 * @param obj 图表对象指针
 * @param ser 数据系列
 * @param array X数组
 * @note 该函数设置数据系列的外部X轴数组
 */
void lv_basechart_set_ext_x_array(lv_obj_t *obj, lv_chart_series_t *ser, lv_coord_t array[]);


/**
 * @brief 获取Y数组
 * @param obj 图表对象指针
 * @param ser 数据系列
 * @return 返回Y数组指针
 * @note 该函数获取数据系列的Y轴数组
 */
lv_coord_t *lv_basechart_get_y_array(const lv_obj_t *obj, lv_chart_series_t *ser);


/**
 * @brief 获取X数组
 * @param obj 图表对象指针
 * @param ser 数据系列
 * @return 返回X数组指针
 * @note 该函数获取数据系列的X轴数组
 */
lv_coord_t *lv_basechart_get_x_array(const lv_obj_t *obj, lv_chart_series_t *ser);


/**
 * @brief 获取按下的点
 * @param obj 图表对象指针
 * @return 返回按下的点ID
 * @note 该函数获取用户按下的数据点ID
 */
uint32_t lv_basechart_get_pressed_point(const lv_obj_t *obj);




/**********************
 *      MACROS
 **********************/

#endif /*LVSF_USE_BASECHART*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _LV_BASECHART_H */
