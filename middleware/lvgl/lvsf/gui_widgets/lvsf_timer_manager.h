/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LVSF_TIMER_MANAGER_H
#define _LVSF_TIMER_MANAGER_H

#include <rtthread.h>
#include "lvgl.h"


/**
 * @brief 定时器对象状态枚举
 */
enum
{
    LV_TIMER_OBJ_PAUSE = 0,           /**<定时器暂停状态*/
    LV_TIMER_OBJ_RUNNING,             /**<定时器运行状态*/
};
typedef uint8_t lv_timer_obj_status_t;

/**
 * @brief 定时器管理对象节点数据结构
 */
typedef struct
{
    void *sign;                         //用于标记的指针，创建和删除需要此标记用于配对
    lv_timer_obj_status_t status;       /**<定时器状态*/
    lv_timer_cb_t timer_cb;             /**<定时器回调函数*/
    rt_list_t obj_list;                 /**<对象列表节点*/
} lv_timer_manager_obj_node_t;

/**
 * @brief 定时器管理节点数据结构
 */
typedef struct
{
    uint32_t count;                     /**<对象计数*/
    lv_timer_t *timer;                  /**<LVGL定时器指针*/
    rt_list_t timer_list;               /**<定时器列表节点*/
    rt_list_t obj_list;                 /**<对象列表节点*/
} lv_timer_manager_node_t;


/**
 * @brief 设置定时器周期
 * @param sign 定时器标记指针
 * @param period 周期值（毫秒）
 */
void lv_timer_manager_set_period(void *sign, uint32_t period);

/**
 * @brief 启动定时器管理器
 * @note 启动所有已注册的定时器
 */
void lv_timer_manager_start(void);

/**
 * @brief 停止定时器管理器
 * @note 停止所有已注册的定时器
 */
void lv_timer_manager_stop(void);

/**
 * @brief 移除指定标记的定时器
 * @param sign 定时器标记指针
 */
void lv_timer_manager_remove(void *sign);

/**
 * @brief 移除所有定时器
 */
void lv_timer_manager_remove_all(void);

/**
 * @brief 插入新的定时器
 * @param sign 定时器标记指针
 * @param period 定时器周期（毫秒）
 * @param timer_cb 定时器回调函数
 */
void lv_timer_manager_insert(void *sign, uint32_t period, lv_timer_cb_t timer_cb);

/**
 * @brief 设置定时器对象状态
 * @param sign 定时器标记指针
 * @param status 目标状态（运行/暂停）
 */
void lv_timer_manager_set_obj_status(void *sign, lv_timer_obj_status_t status);

#endif /* _LVSF_TIMER_MANAGER_H */
