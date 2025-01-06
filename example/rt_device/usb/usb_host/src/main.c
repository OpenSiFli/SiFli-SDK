#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "dfs_file.h"
#include "drv_flash.h"


int main(void)
{
    HAL_RCC_EnableModule(RCC_MOD_USBC);
    /* Output a message on console using printf function */
    rt_kprintf("Use help to check USB host command!\n");
    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);    // Let system breath.
    }
    return 0;
}