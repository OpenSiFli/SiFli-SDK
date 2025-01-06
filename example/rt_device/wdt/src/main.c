#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "time.h"
#include <rtdevice.h>

/* Common functions for RT-Thread based platform -----------------------------------------------*/


/* User code start from here --------------------------------------------------------*/


// extern void rt_hw_watchdog_init(void);

/**
 * @brief This function is invoked in WDT_IRQHandler.
 *        It can be overidden to do some work when WDT1 timeout occured.
 *        Ex. to store exception context and reboot immediately.
 */
void wdt_store_exception_information(void)
{
    rt_kprintf("WDT1 timeout occurs.\n");
    rt_kprintf("WDT reconfig:\n");
    rt_kprintf("  WDT1 and WDT2 has been stopped.\n");
    rt_kprintf("  IWDT refreshed and set timeout time to WDT_TIMEOUT.\n");
    return;
}

/**
 * @brief WDT ON/OFF
 * @param en 0: OFF 1: ON
 */
static void watchdog_set_status(uint8_t en)
{
#ifdef RT_USING_WDT
    /* Set wdt status 0. */
    rt_hw_watchdog_set_status(en);
    /* Avoid repeat set hook. */
    rt_hw_watchdog_hook(0);
    if (!en)
    {
        /* Stop wdt. */
        rt_hw_watchdog_deinit();
    }
    else
    {
        /* Set hook for watchdog petting. */
        rt_hw_watchdog_hook(1);
    }
#endif
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_kprintf("\nWDT Example.\n");
    /* WDT initialization. */
    /**
     * rt_hw_watchdog_init invoked by default.
     * Operations which rt_hw_watchdog_init performs:
     * 1. register "wdt" device.
     * 2. find "wdt" device : wdt_dev = rt_device_find("wdt");
     * 3. open "wdt" device : rt_device_open(wdt_dev, RT_DEVICE_FLAG_RDWR)
     * 4. set wdt timeout : rt_device_control(wdt_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &count);
     * 5. register idle hook for watchdog feeding : rt_hw_watchdog_hook(1);
     */
    // rt_hw_watchdog_init();

    /* Diable WDT. */
    watchdog_set_status(0);
    rt_kprintf("WDT off.\n");
    /* Enable WDT. */
    watchdog_set_status(1);
    rt_kprintf("WDT on.\n");
    /* Cancel feeding the dog in idle thread. */
    rt_hw_watchdog_hook(0);
    rt_kprintf("Unregister idle hook.\n");

    /* Infinite loop */
    while (1)
    {
        static uint32_t count = 0;

        /* Watchdog petting per 5 seconds. */
        rt_thread_mdelay(5000);
        rt_kprintf("watchdog feeding.\n");
        rt_hw_watchdog_pet();

        /* Stop watchdog petting after 10 times. */
        if (++ count >= 10)
        {
            rt_kprintf("Stop watchdog feeding.\n");
            while (1) {};
        }
    }
    return 0;
}

