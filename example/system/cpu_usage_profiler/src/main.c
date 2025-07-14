#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include <rtdevice.h>


/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{

    /* Infinite loop */
    while (1)
    {
        /* Get system time per second. */
        rt_thread_mdelay(1000);
    }
    return 0;
}

