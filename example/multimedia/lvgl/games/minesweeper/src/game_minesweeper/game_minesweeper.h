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
     * @brief 扫雷小游戏 FOR SF32
     * @author 启凡科创
     * @date 2025-06-11
     * @version 1.2
     * @copyright OwO
     */

    LV_FONT_DECLARE(game_minesweeper_font_16);

#ifndef MOUSE_SENSITY_DEFAULT
#define MOUSE_SENSITY_DEFAULT 0.7 // 默认鼠标灵敏度
#endif
#define MOUSE_SHAKE_AREA 3 // 防抖范围
#ifndef OLED_RADIUS_PIX
#define OLED_RADIUS_PIX 60 // 圆角半径像素点个数
#endif

#define MINESWEEPER_MEMERY_MALLOC(size) malloc(size)       // 申请内存函数
#define MINESWEEPER_MEMERY_FREE(x) free(x)                 // 释放内存函数
#define MINESWEEPER_GET_MS_TIC (rt_tick_get_millisecond()) // 获取系统ms时间

    void game_minesweeper_load(void);
    void game_minesweeper_shutdown(void);

#ifdef __cplusplus
}
#endif
