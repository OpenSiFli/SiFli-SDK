#include <rtthread.h>

/**
 * @brief  QEMU hello world: pure RT-Thread application, no SiFli HAL usage.
 * @param  None
 * @retval 0 if success, otherwise failure number
 */
int main(void)
{
    /* Output a message on the QEMU semihosting console */
    rt_kprintf("Hello SiFli on QEMU!\n");

    /* Infinite loop, yield CPU time to other threads */
    while (1)
    {
        rt_thread_mdelay(1000);
    }
    return 0;
}
