#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif
#include <stdlib.h>
#include "rtthread.h"

    /**
     * @brief 扫雷小游戏minesweeper FOR SF32
     * @author 启凡科创 QFTEK
     * @date 2025-09-24
     * @version 1.2
     * @copyright OwO
     */

#ifndef MINESWEEPER_LANGUAGE_DEFAULT
#define MINESWEEPER_LANGUAGE_DEFAULT 1 // 默认语言 Default language 0:中文 1:English
#endif

#ifndef MOUSE_SENSITY_DEFAULT
#define MOUSE_SENSITY_DEFAULT 0.7 // 默认鼠标灵敏度 Mouse sensitivity
#endif
#define MOUSE_SHAKE_AREA 3 // 防抖范围 Debounce range
#ifndef AMOLED_RADIUS_PIX
#define AMOLED_RADIUS_PIX 60 // 屏幕圆角半径像素点个数  Number of pixel points for screen corner radius
#endif

#define MINESWEEPER_MEMERY_MALLOC(size) malloc(size)       // 申请内存函数 Apply for memory function
#define MINESWEEPER_MEMERY_FREE(x) free(x)                 // 释放内存函数 Memory release function
#define MINESWEEPER_GET_MS_TIC (rt_tick_get_millisecond()) // 获取系统ms时间 Get the system ms time

    void game_minesweeper_install(void);
    void game_minesweeper_shutdown(void);

#ifdef __cplusplus
}
#endif
