/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*********************
 *      INCLUDES
 *********************/
#ifndef _LV_BASELABEL_H
#define _LV_BASELABEL_H

#ifdef __cplusplus
extern "C" {
#endif


/*********************
    *      INCLUDES
*********************/
#include "lvsf_conf_internal.h"
#if LVSF_USE_BASELABEL != 0

#if !defined(PY_GEN)
#include "lv_ext_resource_manager.h"
#endif

#include "lvsf_obj_ext.h"

extern const lv_obj_class_t lv_baselabel_class;

/*Testing of dependencies*/
#if LV_USE_LABEL == 0
#error "lvsf_baselabel: lv_label is required. Enable it in lv_conf.h (LV_USE_LABEL  1) "
#endif


/*********************
 *      DEFINES
 *********************/

/**
 * @brief 基础标签刷新回调函数类型定义
 * @param obj 对象指针
 * @param id_tab 数据源ID数组指针
 * @param id_num 数据源ID数量
 * @return 返回32位整型数据
 * @note 该回调函数用于从应用程序工具获取数据，更新标签显示内容
 */
typedef int32_t (*lv_baselabel_refresh_cb)(struct _lv_obj_t *obj, uint32_t *id_tab, uint8_t id_num);

/**********************
 *      TYPEDEFS
 **********************/

/**
 * @brief 基础标签数据结构
 * @note 该结构体继承自lv_label_t，扩展了基础标签的功能
 */
typedef struct
{
    lv_label_t ext;

    uint16_t    cus_text_len;   /*用户自定义文本长度*/
    char        *cus_text;      /*用户自定义要显示的文本*/
#if !defined(PY_GEN)
    lv_ext_str_id_t *str_tab;   /*字符串ID表（用于周、月等字符串显示）*/
#endif
    /*use for app tool*/
    uint8_t sfat_str_num;       /*静态字符串数量*/
    uint8_t sfat_str_index;     /*当前显示的静态字符串索引*/
    char **sfat_str_tab;        /*静态字符串数组指针*/
} lv_baselabel_t;


/**
 * @brief 基础标签刷新定时器回调函数
 * @param timer 定时器对象指针
 * @note 该函数通过定时器定期调用用户注册的回调函数来更新标签文本内容
 *       如果设置了原始位置坐标，还会将标签对齐到原始位置
 */
void lv_baselabel_refresh_timer(lv_timer_t *timer);


/**
 * @brief 创建基础标签对象
 * @param parent 父对象指针
 * @return 成功返回标签对象指针，失败返回NULL
 * @note 该函数创建一个lv_baselabel对象并初始化
 */
lv_obj_t *lv_baselabel_create(lv_obj_t *parent);


/**
 * @brief 设置自定义文本内容
 * @param label 标签对象指针
 * @param text 文本内容指针（不能为空）
 * @note 该函数会动态分配内存存储自定义文本，文本会被复制到标签的cus_text字段
 */
void lv_baselabel_set_custom_text(lv_obj_t *label, const char *text);


#if !defined(PY_GEN)

/**
 * @brief 使用格式化字符串设置标签文本
 * @param obj 标签对象指针
 * @param fmt 格式化字符串
 * @param ... 可变参数列表
 * @note 该函数支持格式化字符串，类似于printf的用法
 */
void lv_baselabel_set_text_fmt(lv_obj_t *obj, const char *fmt, ...);// LV_FORMAT_ATTRIBUTE(2, 3);

#endif


/**
 * @brief 设置标签显示的文本
 * @param obj 标签对象指针
 * @param text 文本内容指针
 * @note 该函数直接调用底层lv_label_set_text函数设置文本
 */
void lv_baselabel_set_text(lv_obj_t *obj, const char *text);


/**
 * @brief 设置静态文本（不复制内容）
 * @param obj 标签对象指针
 * @param text 文本内容指针
 * @note 该函数设置文本时不复制内容，仅保存指针，适用于常量字符串
 */
void lv_baselabel_set_text_static(lv_obj_t *obj, const char *text);


/**
 * @brief 设置长文本显示模式
 * @param obj 标签对象指针
 * @param long_mode 长文本模式（lv_label_long_mode_t类型）
 * @note 该函数设置当文本过长时的显示方式（如换行、滚动、省略等）
 */
void lv_baselabel_set_long_mode(lv_obj_t *obj, lv_label_long_mode_t long_mode);


/**
 * @brief 设置是否启用颜色重绘功能
 * @param obj 标签对象指针
 * @param en 启用标志（true启用，false禁用）
 * @note 启用后可以在文本中使用颜色标记来改变部分文本的颜色
 */
void lv_baselabel_set_recolor(lv_obj_t *obj, bool en);


/**
 * @brief 设置文本选择起始位置
 * @param obj 标签对象指针
 * @param index 选择起始字符索引
 * @note 该函数设置文本选择的起始位置，用于文本选择功能
 */
void lv_baselabel_set_text_sel_start(lv_obj_t *obj, uint32_t index);


/**
 * @brief 设置文本选择结束位置
 * @param obj 标签对象指针
 * @param index 选择结束字符索引
 * @note 该函数设置文本选择的结束位置，用于文本选择功能
 */
void lv_baselabel_set_text_sel_end(lv_obj_t *obj, uint32_t index);


/**
 * @brief 获取标签显示的文本
 * @param obj 标签对象指针
 * @return 返回指向文本内容的指针
 */
char *lv_baselabel_get_text(const lv_obj_t *obj);


/**
 * @brief 获取长文本显示模式
 * @param obj 标签对象指针
 * @return 返回当前的长文本模式
 */
lv_label_long_mode_t lv_baselabel_get_long_mode(const lv_obj_t *obj);


/**
 * @brief 获取颜色重绘功能状态
 * @param obj 标签对象指针
 * @return 返回true表示启用颜色重绘，false表示禁用
 */
bool lv_baselabel_get_recolor(const lv_obj_t *obj);


/**
 * @brief 获取指定字符的位置坐标
 * @param obj 标签对象指针
 * @param char_id 字符索引
 * @param pos 用于返回位置坐标的指针（lv_point_t类型）
 * @note 该函数获取指定字符在标签中的x、y坐标位置
 */
void lv_baselabel_get_letter_pos(const lv_obj_t *obj, uint32_t char_id, lv_point_t *pos);


/**
 * @brief 获取指定坐标处的字符索引
 * @param obj 标签对象指针
 * @param pos_in 输入的坐标指针（lv_point_t类型）
 * @return 返回坐标处的字符索引，如果坐标不在文本范围内则返回LV_LABEL_POS_NONE
 */
uint32_t lv_baselabel_get_letter_on(const lv_obj_t *obj, lv_point_t *pos_in);


/**
 * @brief 检查指定坐标处是否有一个字符
 * @param obj 标签对象指针
 * @param pos 坐标指针（lv_point_t类型）
 * @return 返回true表示坐标处有字符，false表示没有
 */
bool lv_baselabel_is_char_under_pos(const lv_obj_t *obj, lv_point_t *pos);


/**
 * @brief 获取文本选择起始位置
 * @param obj 标签对象指针
 * @return 返回选中文本的起始字符索引
 */
uint32_t lv_baselabel_get_text_selection_start(const lv_obj_t *obj);


/**
 * @brief 获取文本选择结束位置
 * @param obj 标签对象指针
 * @return 返回选中文本的结束字符索引
 */
uint32_t lv_baselabel_get_text_selection_end(const lv_obj_t *obj);


/**
 * @brief 在指定位置插入文本
 * @param obj 标签对象指针
 * @param pos 插入位置的字符索引
 * @param txt 要插入的文本内容
 * @note 该函数在指定位置插入新文本，原位置及之后的文本会向后移动
 */
void lv_baselabel_ins_text(lv_obj_t *obj, uint32_t pos, const char *txt);


/**
 * @brief 剪切（删除）文本
 * @param obj 标签对象指针
 * @param pos 删除起始位置的字符索引
 * @param cnt 要删除的字符数量
 * @note 该函数从指定位置开始删除指定数量的字符
 */
void lv_baselabel_cut_text(lv_obj_t *obj, uint32_t pos, uint32_t cnt);


/**
 * @brief 设置带省略号的文本
 * @param obj 标签对象指针
 * @param text 文本内容指针
 * @param display_w 显示区域宽度（像素）
 * @param ellip_en 是否启用省略号（true启用，false禁用）
 * @note 当文本长度超过显示宽度时，如果启用省略号会在末尾显示"..."表示被截断
 */
void lv_baselabel_set_ellip_txt(lv_obj_t *obj, const char *text, uint16_t display_w, bool ellip_en);


/**
 * @brief 添加静态字符串到标签
 * @param label 标签对象指针
 * @param str 字符串内容指针
 * @param len 字符串长度
 * @note 该函数会动态分配内存存储字符串，并增加sfat_str_num计数
 *       字符串会被复制到内部数组中
 */
void lv_baselabel_add_sfat_str(lv_obj_t *label, char *str, uint8_t len);


/**
 * @brief 清除所有静态字符串
 * @param label 标签对象指针
 * @note 该函数会释放所有已添加的静态字符串内存，并重置计数器
 */
void lv_baselabel_clear_sfat_str(lv_obj_t *label);


/**
 * @brief 选择并显示静态字符串
 * @param label 标签对象指针
 * @param index 字符串索引（0-based）
 * @note 该函数根据索引从静态字符串数组中选择一个字符串并显示
 *       如果索引无效或数组为空则不执行任何操作
 * @attention 索引超出范围时不会报错，但也不会显示任何内容
 * @attention 该函数仅设置显示内容，不会复制字符串
 */
void lv_baselabel_select_sfat_str(lv_obj_t *label, uint8_t index);


/**
 * @brief 获取静态字符串数量
 * @param label 标签对象指针
 * @return 返回已添加的静态字符串数量
 * @note 该函数返回值为0表示尚未添加任何静态字符串
 */
uint8_t lv_baselabel_get_sfat_str_num(lv_obj_t *label);


/**
 * @brief 获取当前显示的静态字符串索引
 * @param label 标签对象指针
 * @return 返回当前显示的静态字符串索引
 * @note 索引范围为0到(sfat_str_num-1)
 * @note 清除静态字符串后索引会重置为0
 */
uint8_t lv_baselabel_get_select_sfat_str_idx(lv_obj_t *label);




/**********************
 *      MACROS
 **********************/

#endif /*LVSF_USE_BASELABEL*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _LV_BASELABEL_H */
