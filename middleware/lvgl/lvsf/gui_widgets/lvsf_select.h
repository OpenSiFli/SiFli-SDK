/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LVSF_SELECT_H_
#define LVSF_SELECT_H_

#ifdef __cplusplus
extern "C" {
#endif
/*********************
 *      INCLUDES
 *********************/
#if LVSF_USE_SELECT != 0

/*Testing of dependencies*/
#if LV_USE_BTN == 0
#error "lvsf_select: lv_btn is required. Enable it in lv_conf.h (LV_USE_BTN  1) "
#endif

#if LV_USE_LABEL == 0
#error "lvsf_select: lv_label is required. Enable it in lv_conf.h (LV_USE_LABEL  1) "
#endif

#if LV_USE_IMG == 0
#error "lvsf_select: lv_img is required. Enable it in lv_conf.h (LV_USE_IMG  1) "
#endif

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
/** Mac items. */

#define LV_SELECT_ELE_INTERVAL    4

/**
 * @brief 选择控件类型枚举
 */
enum
{
    LV_SELECT_TYPE_SINGLE,            /**<单选模式*/
    LV_SELECT_TYPE_MULTI,             /**<多选模式*/
};
typedef uint8_t lv_select_type_t;

/**
 * @brief 选择控件状态枚举
 */
enum
{
    LV_SELECT_STATE_UNCHECK,          /**<未选中状态*/
    LV_SELECT_STATE_CHECK,            /**<选中状态*/
    LV_SELECT_STATE_DISABLE,          /**<禁用状态*/
};
typedef uint8_t lv_select_state_t;

/**
 * @brief 选择控件元素数据结构
 */
typedef struct
{
    lv_select_state_t state;          /**<元素状态*/
    lv_obj_t *bg_obj;                 /**<背景对象*/
    lv_obj_t *icon;                   /**<图标对象*/
} lv_select_ele_t;

/**
 * @brief 选择控件头数据结构
 */
typedef struct
{
    lv_obj_t    ext;                  /**<扩展对象*/
    uint32_t    type : 1;             //lv_select_type_t 选择类型
    lv_coord_t          w;            /**<元素宽度*/
    lv_coord_t          h;            /**<元素高度*/

    uint16_t            ele_num;      /**<元素数量*/
    uint16_t            select_idx;   /**<当前选中索引*/

    lv_select_ele_t     *ele_tab;     /**<元素数组*/

    const void *check_src;            /**<选中图标源*/
    const void *uncheck_src;          /**<未选中图标源*/
} lv_select_t;



/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * @brief 创建选择控件
 * @param par 父对象指针
 * @return 选择控件对象指针
 */
lv_obj_t *lv_select_create(lv_obj_t *par);

/**
 * @brief 设置选择控件类型
 * @param select 选择控件对象指针
 * @param type 选择类型（单选/多选）
 */
void lv_select_set_type(lv_obj_t *select, lv_select_type_t type);

/**
 * @brief 设置选择控件元素数量
 * @param select 选择控件对象指针
 * @param num 元素数量
 */
void lv_select_set_ele_num(lv_obj_t *select, uint16_t num);

/**
 * @brief 设置选择控件元素大小
 * @param select 选择控件对象指针
 * @param w 元素宽度
 * @param h 元素高度
 */
void lv_select_set_ele_size(lv_obj_t *select, lv_coord_t w, lv_coord_t h);

/**
 * @brief 设置选择控件元素状态
 * @param select 选择控件对象指针
 * @param idx 元素索引
 * @param state 元素状态
 */
void lv_select_set_ele_state(lv_obj_t *select, uint16_t idx, lv_select_state_t state);

/**
 * @brief 设置选择控件选中图标
 * @param select 选择控件对象指针
 * @param src 选中图标源
 */
void lv_select_set_check_src(lv_obj_t *select, const void *src);

/**
 * @brief 设置选择控件未选中图标
 * @param select 选择控件对象指针
 * @param src 未选中图标源
 */
void lv_select_set_uncheck_src(lv_obj_t *select, const void *src);

/**
 * @brief 获取选择控件当前选中索引
 * @param select 选择控件对象指针
 * @return 选中索引值
 */
uint16_t lv_select_get_select_idx(lv_obj_t *select);

/**
 * @brief 获取选择控件指定索引的元素
 * @param select 选择控件对象指针
 * @param idx 元素索引
 * @return 元素对象指针
 */
lv_obj_t *lv_select_get_ele(lv_obj_t *select, uint16_t idx);

/**
 * @brief 获取元素在选择控件中的索引
 * @param select 选择控件对象指针
 * @param ele 元素对象指针
 * @return 元素索引值
 */
uint16_t lv_select_get_ele_idx(lv_obj_t *select, lv_obj_t *ele);

/**
 * @brief 获取选择控件元素数量
 * @param select 选择控件对象指针
 * @return 元素数量
 */
uint16_t lv_select_get_ele_num(lv_obj_t *select);

/**
 * @brief 获取选择控件指定元素的状态
 * @param select 选择控件对象指针
 * @param idx 元素索引
 * @return 元素状态
 */
lv_select_state_t lv_select_get_ele_state(lv_obj_t *select, uint16_t idx);



/**********************
 *      MACROS
 **********************/

#endif /*LVSF_USE_SELECT*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LVSF_SELECT_H*/
