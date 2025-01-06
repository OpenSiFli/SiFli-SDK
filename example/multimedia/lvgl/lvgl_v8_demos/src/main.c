#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "littlevgl2rtt.h"
#include "lv_ex_data.h"
#include "lv_demos.h"

#ifdef  LV_USE_DEMO_WIDGETS
    #define lv_demo_main lv_demo_widgets
#elif   defined(LV_USE_DEMO_BENCHMARK)
    #define lv_demo_main lv_demo_benchmark
#elif   defined(LV_USE_DEMO_KEYPAD_AND_ENCODER)
    #define lv_demo_main lv_demo_keypad_encoder
#elif   defined(LV_USE_DEMO_MUSIC)
    #define lv_demo_main lv_demo_music
#elif   defined(LV_USE_DEMO_STRESS)
    #define lv_demo_main  lv_demo_stress
#else
    #error "Select a demo application to start"
#endif


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

    lv_demo_main();

    while (1)
    {
        ms = lv_task_handler();
        rt_thread_mdelay(ms);
    }
    return RT_EOK;

}
