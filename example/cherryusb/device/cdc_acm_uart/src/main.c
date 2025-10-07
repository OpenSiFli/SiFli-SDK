/*
 * USB-UART Bridge Main Application
 */
#include "rtthread.h"
#include "bf0_hal.h"

#define DBG_TAG "main"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>
extern void cdc_acm_init(uint8_t busid, uintptr_t base);

/**
 * @brief  Main program entry point
 * @param  None
 * @retval RT_EOK if success
 */
int main(void)
{
    LOG_I("USB-UART Bridge starting...");
    
    /* Initialize USB CDC ACM bridge */
    cdc_acm_init(0, (uintptr_t)USBC_BASE);

    LOG_I("USB-UART Bridge ready");
    
    /* Main loop - keep application running */
    while (1) {
        rt_thread_mdelay(1000);
    }
    
    return RT_EOK;
}