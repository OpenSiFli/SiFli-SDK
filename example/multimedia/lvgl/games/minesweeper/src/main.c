#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "littlevgl2rtt.h"
#include "lv_ex_data.h"

/*
    如需要对默认语言、灵敏度、屏幕圆角参数等进行自定义配置，可跳转到此头文件内进行修改配置

    If you need to customize the default language, sensitivity, screen corner parameters, etc.,
    you can jump to this header file to modify the configuration
*/
#include "game_minesweeper.h"

/**
 * @brief 扫雷小游戏minesweeper for SF32
 * @author 启凡科创 QFTEK
 * @date 2025-09-24
 * @version 1.2
 * @copyright OwO
 */

/**
 * @brief  Main program
 * @param  None
 * @retval 0 if success, otherwise failure number
 */
int main(void)
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t ms;

    /* init littlevGL */
    ret = littlevgl2rtt_init("lcd");
    if (ret != RT_EOK)
    {
        return ret;
    }
    lv_ex_data_pool_init();

    // install game
    game_minesweeper_install();

    while (1)
    {
        ms = lv_task_handler();
        rt_thread_mdelay(ms);
    }
    return RT_EOK;
}
