#include "rtthread.h"
#include "rtdevice.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"


/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    /*recode led state*/

    /* Output a message on console using printf function */
    rt_kprintf("Start example blink!\n");

    /*Init pin state */
    rt_pin_mode(BSP_LED1_PIN, PIN_MODE_OUTPUT);
    bool state = RT_FALSE;

    /* Infinite loop, len flash inter 1s */
    while (1)
    {
        rt_kprintf("Turning the LED %s;\n", state == RT_TRUE ? "ON" : "OFF");
#ifndef BSP_LED1_ACTIVE_HIGH
        rt_pin_write(BSP_LED1_PIN, !state);
#else
        rt_pin_write(BSP_LED1_PIN, state);
#endif
        /*toggle the LED statu*/
        state = !state;
        rt_thread_mdelay(1000);
    }
    return 0;
}

