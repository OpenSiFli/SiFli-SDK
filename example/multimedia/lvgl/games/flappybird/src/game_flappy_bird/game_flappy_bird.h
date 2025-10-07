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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

    /**
     * @brief 笨笨鸟小游戏 flappy bird FOR SF32
     * @author 启凡科创 QFTEK
     * @date 2025-10-07
     * @version 1.0
     * @copyright OwO
     */

    void game_flappy_bird_install(); // INSTALL GAME

#ifdef __cplusplus
}
#endif
