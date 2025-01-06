#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "littlevgl2rtt.h"
#include "lv_ex_data.h"
#include "lv_examples.h"


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

    /* add example code here.
       example code locates in folder external/lvgl/examples */
    lv_example_scroll_1();
    //lv_example_grid_1();
    while (1)
    {
        ms = lv_task_handler();
        rt_thread_mdelay(ms);
    }
    return RT_EOK;

}
